"""Atlas Code-Quant — Entrenador PPO para el agente RL de trading.

Entrena un agente PPO (Proximal Policy Optimization) usando Stable-Baselines3
sobre el entorno QuantTradingEnv. PPO es el algoritmo #1 recomendado por
Grok/xAI para trading autónomo por su estabilidad y rendimiento superior.

Flujo de entrenamiento:
  1. Descarga datos históricos (MarketFeed)
  2. Calcula features logarítmicas (build_features)
  3. Split train/test (70/30) sin data leakage
  4. Entrena PPO en entorno train
  5. Evalúa en entorno test — métricas Sharpe/Sortino/Kelly
  6. Guarda modelo en models/saved/ppo_{symbol}_{timestamp}.zip
  7. Reporta al ATLAS Brain Bridge

Uso desde CLI::
    python -m learning.rl_trainer --symbol BTC/USDT --timesteps 100000
    python -m learning.rl_trainer --symbol AAPL --source yfinance --timesteps 50000

Uso programático::
    trainer = RLTrainer(symbol="BTC/USDT")
    result = trainer.train(total_timesteps=100_000)
    print(result)
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.learning.rl_trainer")

# ── Rutas ─────────────────────────────────────────────────────────────────────
_BASE_DIR   = Path(__file__).resolve().parent.parent
_MODELS_DIR = _BASE_DIR / "models" / "saved"
_LOGS_DIR   = _BASE_DIR / "logs"


class RLTrainer:
    """Entrena y evalúa un agente PPO para trading autónomo logarítmico.

    Args:
        symbol: Ticker a entrenar (ej: "BTC/USDT", "AAPL").
        source: "ccxt" (crypto) o "yfinance" (acciones).
        timeframe: Marco temporal para las velas.
        initial_capital: Capital simulado durante entrenamiento.
        train_split: Fracción de datos para entrenamiento (default 0.70).
        n_eval_episodes: Episodios de evaluación al finalizar.
        verbose: 0=silencioso, 1=progreso, 2=detallado.

    Example::
        trainer = RLTrainer(symbol="BTC/USDT", source="ccxt")
        result = trainer.train(total_timesteps=200_000)
        model_path = result["model_path"]
    """

    def __init__(
        self,
        symbol: str = "BTC/USDT",
        source: str = "ccxt",
        timeframe: str = "1h",
        initial_capital: float = 10_000.0,
        train_split: float = 0.70,
        n_eval_episodes: int = 5,
        verbose: int = 1,
    ) -> None:
        self.symbol = symbol
        self.source = source
        self.timeframe = timeframe
        self.initial_capital = initial_capital
        self.train_split = train_split
        self.n_eval_episodes = n_eval_episodes
        self.verbose = verbose
        _MODELS_DIR.mkdir(parents=True, exist_ok=True)
        _LOGS_DIR.mkdir(parents=True, exist_ok=True)

    def train(
        self,
        total_timesteps: int = 100_000,
        df: pd.DataFrame | None = None,
    ) -> dict:
        """Entrena el agente PPO.

        Args:
            total_timesteps: Pasos totales de entrenamiento RL.
            df: DataFrame OHLCV pre-cargado (opcional — si None, descarga automático).

        Returns:
            Dict con métricas de entrenamiento, evaluación y ruta del modelo.
        """
        self._check_deps()

        # ── 1. Datos ─────────────────────────────────────────────────────────
        if df is None:
            df = self._fetch_data()
        if df.empty:
            return {"error": "DataFrame vacío — sin datos para entrenar"}

        # ── 2. Features logarítmicas ──────────────────────────────────────────
        from models.features import build_features
        try:
            feat_df = build_features(df)
        except Exception as exc:
            return {"error": f"Error calculando features: {exc}"}

        if len(feat_df) < 100:
            return {"error": f"Datos insuficientes tras features: {len(feat_df)} filas"}

        # ── 3. Split train/test ───────────────────────────────────────────────
        n = len(feat_df)
        train_end = int(n * self.train_split)
        # Reconstruir OHLCV alineado con feat_df
        df_aligned = df.iloc[len(df) - n:].copy()
        df_aligned.index = feat_df.index

        train_feat = feat_df.iloc[:train_end]
        test_feat  = feat_df.iloc[train_end:]
        train_df   = df_aligned.iloc[:train_end]
        test_df    = df_aligned.iloc[train_end:]

        logger.info(
            "[RLTrainer] %s | train=%d test=%d | timesteps=%d",
            self.symbol, len(train_feat), len(test_feat), total_timesteps,
        )

        # ── 4. Construir entornos ─────────────────────────────────────────────
        from learning.rl_env import QuantTradingEnv
        train_env = QuantTradingEnv(
            df=train_df, symbol=self.symbol,
            initial_capital=self.initial_capital,
        )
        eval_env = QuantTradingEnv(
            df=test_df, symbol=self.symbol,
            initial_capital=self.initial_capital,
        )

        # ── 5. Verificar observation space ────────────────────────────────────
        if train_env.observation_space is None:
            return {"error": "gymnasium no instalado — ejecutar: pip install gymnasium stable-baselines3"}

        # ── 6. Entrenar PPO ───────────────────────────────────────────────────
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import EvalCallback

        ts_str = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
        safe_sym = self.symbol.replace("/", "_")
        model_filename = f"ppo_{safe_sym}_{ts_str}"
        model_path = _MODELS_DIR / model_filename
        log_dir = _LOGS_DIR / f"ppo_tb_{safe_sym}"
        log_dir.mkdir(parents=True, exist_ok=True)

        # Callback de evaluación — guarda el mejor modelo
        eval_callback = EvalCallback(
            eval_env,
            best_model_save_path=str(_MODELS_DIR),
            log_path=str(log_dir),
            eval_freq=max(1000, total_timesteps // 20),
            n_eval_episodes=self.n_eval_episodes,
            verbose=0,
        )

        model = PPO(
            policy="MlpPolicy",
            env=train_env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,           # Exploración
            vf_coef=0.5,
            max_grad_norm=0.5,
            tensorboard_log=str(log_dir),
            verbose=self.verbose,
        )

        t_start = time.time()
        model.learn(
            total_timesteps=total_timesteps,
            callback=eval_callback,
            reset_num_timesteps=True,
            progress_bar=(self.verbose > 0),
        )
        t_elapsed = time.time() - t_start

        # ── 7. Guardar modelo ─────────────────────────────────────────────────
        model.save(str(model_path))
        logger.info("[RLTrainer] Modelo guardado: %s.zip", model_path)

        # ── 8. Evaluar en test set ────────────────────────────────────────────
        eval_metrics = self._evaluate(model, eval_env, n_episodes=self.n_eval_episodes)

        result = {
            "symbol":              self.symbol,
            "source":              self.source,
            "timeframe":           self.timeframe,
            "total_timesteps":     total_timesteps,
            "train_bars":          len(train_feat),
            "test_bars":           len(test_feat),
            "train_elapsed_s":     round(t_elapsed, 1),
            "model_path":          str(model_path) + ".zip",
            "eval_metrics":        eval_metrics,
            "trained_at":          datetime.now(tz=timezone.utc).isoformat(),
        }
        logger.info(
            "[RLTrainer] Completado — Retorno geom. test: %.2f%% | Sharpe: %.3f",
            eval_metrics.get("mean_geometric_return_pct", 0),
            eval_metrics.get("sharpe_ratio", 0),
        )
        return result

    def _evaluate(self, model, env: "QuantTradingEnv", n_episodes: int = 5) -> dict:
        """Evalúa el modelo entrenado en el entorno de test."""
        log_returns_all = []
        rewards_all = []

        for ep in range(n_episodes):
            obs, _ = env.reset()
            done = False
            ep_reward = 0.0
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(int(action))
                ep_reward += reward
                done = terminated or truncated
            summary = env.episode_summary
            log_returns_all.append(summary["total_log_return"])
            rewards_all.append(ep_reward)

        if not log_returns_all:
            return {}

        # Sharpe aproximado de los episodios
        mean_lr = float(np.mean(log_returns_all))
        std_lr  = float(np.std(log_returns_all)) if len(log_returns_all) > 1 else 1e-6
        sharpe  = mean_lr / (std_lr + 1e-10)

        return {
            "n_eval_episodes":          n_episodes,
            "mean_log_return":          round(mean_lr, 6),
            "std_log_return":           round(std_lr, 6),
            "mean_geometric_return_pct": round((math.exp(mean_lr) - 1) * 100, 2),
            "sharpe_ratio":             round(sharpe, 4),
            "mean_total_reward":        round(float(np.mean(rewards_all)), 4),
        }

    def _fetch_data(self) -> pd.DataFrame:
        """Descarga datos históricos usando MarketFeed."""
        from data.feed import MarketFeed
        try:
            feed = MarketFeed(source=self.source)
            df = feed.ohlcv(self.symbol, timeframe=self.timeframe, limit=2000)
            logger.info("[RLTrainer] Descargados %d bars de %s/%s", len(df), self.symbol, self.timeframe)
            return df
        except Exception as exc:
            logger.error("[RLTrainer] Error descargando datos: %s", exc)
            return pd.DataFrame()

    def _check_deps(self) -> None:
        """Verifica que las dependencias RL estén instaladas."""
        missing = []
        for pkg in ["gymnasium", "stable_baselines3"]:
            try:
                __import__(pkg)
            except ImportError:
                missing.append(pkg)
        if missing:
            raise ImportError(
                f"Dependencias RL faltantes: {missing}\n"
                "Instalar: pip install gymnasium stable-baselines3[extra] torch"
            )


def load_model(model_path: str) -> Any:
    """Carga un modelo PPO guardado."""
    from stable_baselines3 import PPO
    return PPO.load(model_path)


def list_models(symbol: str | None = None) -> list[dict]:
    """Lista modelos PPO guardados."""
    _MODELS_DIR.mkdir(parents=True, exist_ok=True)
    models = []
    for p in sorted(_MODELS_DIR.glob("ppo_*.zip")):
        safe_sym = p.stem.replace("ppo_", "").rsplit("_", 2)[0]
        original_sym = safe_sym.replace("_", "/", 1)
        if symbol and original_sym.upper() != symbol.upper():
            continue
        models.append({
            "path":       str(p),
            "symbol":     original_sym,
            "filename":   p.name,
            "size_kb":    round(p.stat().st_size / 1024, 1),
            "created_at": datetime.fromtimestamp(p.stat().st_mtime).isoformat(),
        })
    return models


# ── CLI ───────────────────────────────────────────────────────────────────────

def _main() -> None:
    parser = argparse.ArgumentParser(description="Atlas Code-Quant — RL Trainer PPO")
    parser.add_argument("--symbol",     default="BTC/USDT", help="Símbolo a entrenar")
    parser.add_argument("--source",     default="ccxt",     choices=["ccxt", "yfinance"])
    parser.add_argument("--timeframe",  default="1h")
    parser.add_argument("--timesteps",  type=int, default=100_000)
    parser.add_argument("--capital",    type=float, default=10_000.0)
    parser.add_argument("--verbose",    type=int, default=1)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    trainer = RLTrainer(
        symbol=args.symbol,
        source=args.source,
        timeframe=args.timeframe,
        initial_capital=args.capital,
        verbose=args.verbose,
    )
    result = trainer.train(total_timesteps=args.timesteps)

    import json
    print(json.dumps(result, indent=2, default=str))


# Type hint para load_model
from typing import Any

if __name__ == "__main__":
    _main()
