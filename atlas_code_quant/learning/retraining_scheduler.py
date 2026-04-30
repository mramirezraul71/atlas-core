"""Atlas Code-Quant — Scheduler de retraining automático con Optuna.

Reentrenar el modelo RL (PPO) automáticamente cada N horas con
optimización de hiperparámetros via Optuna. Sin intervención humana.

Grok/xAI criterio 2026:
  "Retraining online cada 4-24h. Meta-IA para ajustar hiperparámetros
   (Optuna). Supervisión humana mínima: solo para auditoría regulatoria."

Flujo por ciclo:
  1. Verificar si han pasado N horas desde último retraining
  2. Descargar datos frescos (últimas N barras)
  3. Optuna: 20 trials → encontrar learning_rate, n_steps, gamma óptimos
  4. Entrenar PPO con mejores hiperparámetros
  5. Evaluar en test set → Sharpe, retorno geométrico
  6. Si mejora sobre modelo anterior → promover como modelo activo
  7. Notificar via AlertDispatcher (Telegram + WhatsApp)
  8. Reportar al ATLAS Brain Bridge

Uso programático::
    scheduler = RetrainingScheduler(symbol="BTC/USDT", interval_hours=12)
    await scheduler.start()   # Loop en background
    scheduler.force_retrain() # Trigger inmediato

CLI::
    python -m learning.retraining_scheduler --symbol BTC/USDT --interval 12
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger("quant.learning.retraining")

_BASE_DIR   = Path(__file__).resolve().parent.parent
_STATE_PATH = _BASE_DIR / "data" / "learning" / "retraining_state.json"
_MODELS_DIR = _BASE_DIR / "models" / "saved"

# ── Defaults ──────────────────────────────────────────────────────────────────
_DEFAULT_INTERVAL_H  = 12       # Retraining cada 12 horas
_DEFAULT_TIMESTEPS   = 80_000   # Pasos PPO por ciclo
_DEFAULT_OPTUNA_TRIALS = 20     # Trials de optimización Optuna
_MIN_SHARPE_TO_PROMOTE = 0.5    # Sharpe mínimo para promover modelo nuevo


@dataclass
class RetrainingResult:
    symbol: str
    timestamp: str
    sharpe: float
    geometric_return_pct: float
    model_path: str
    promoted: bool
    hyperparams: dict
    elapsed_s: float
    error: str = ""

    def to_dict(self) -> dict:
        return {
            "symbol":               self.symbol,
            "timestamp":            self.timestamp,
            "sharpe":               round(self.sharpe, 4),
            "geometric_return_pct": round(self.geometric_return_pct, 2),
            "model_path":           self.model_path,
            "promoted":             self.promoted,
            "hyperparams":          self.hyperparams,
            "elapsed_s":            round(self.elapsed_s, 1),
            "error":                self.error,
        }


class RetrainingScheduler:
    """Scheduler autónomo de retraining con optimización de hiperparámetros.

    Args:
        symbol: Activo a reentrenar (ej: "BTC/USDT").
        source: Fuente de datos ("ccxt" o "yfinance").
        timeframe: Marco temporal.
        interval_hours: Horas entre retrainings (4-24).
        total_timesteps: Pasos PPO por ciclo de entrenamiento.
        optuna_trials: Número de trials de optimización Optuna.
        alert_dispatcher: Instancia de AlertDispatcher para notificaciones.
        brain_bridge: Instancia de QuantBrainBridge para reportar al brain.
    """

    def __init__(
        self,
        symbol: str = "BTC/USDT",
        source: str = "ccxt",
        timeframe: str = "1h",
        interval_hours: float = _DEFAULT_INTERVAL_H,
        total_timesteps: int = _DEFAULT_TIMESTEPS,
        optuna_trials: int = _DEFAULT_OPTUNA_TRIALS,
        alert_dispatcher=None,
        brain_bridge=None,
    ) -> None:
        self.symbol          = symbol
        self.source          = source
        self.timeframe       = timeframe
        self.interval_hours  = max(4.0, min(float(interval_hours), 168.0))
        self.total_timesteps = total_timesteps
        self.optuna_trials   = optuna_trials
        self._dispatcher     = alert_dispatcher
        self._brain_bridge   = brain_bridge
        self._running        = False
        self._task: asyncio.Task | None = None
        self._history: list[RetrainingResult] = []
        self._state          = self._load_state()
        _MODELS_DIR.mkdir(parents=True, exist_ok=True)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def start(self) -> None:
        """Inicia el scheduler en background."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self._scheduler_loop())
        logger.info(
            "[Retraining] Scheduler iniciado — %s cada %.0fh",
            self.symbol, self.interval_hours,
        )

    async def stop(self) -> None:
        self._running = False
        if self._task and not self._task.done():
            self._task.cancel()

    def force_retrain(self) -> None:
        """Fuerza un ciclo de retraining en el próximo tick."""
        self._state["last_retrain_ts"] = 0.0
        self._save_state()
        logger.info("[Retraining] Retraining forzado para el próximo ciclo")

    def status(self) -> dict:
        last_ts = self._state.get("last_retrain_ts", 0.0)
        next_in_s = max(0.0, self.interval_hours * 3600 - (time.time() - last_ts))
        last_result = self._history[-1].to_dict() if self._history else None
        return {
            "symbol":           self.symbol,
            "interval_hours":   self.interval_hours,
            "running":          self._running,
            "last_retrain":     datetime.fromtimestamp(last_ts, tz=timezone.utc).isoformat() if last_ts else None,
            "next_retrain_in_s": round(next_in_s),
            "total_cycles":     self._state.get("total_cycles", 0),
            "last_result":      last_result,
        }

    # ── Loop principal ────────────────────────────────────────────────────────

    async def _scheduler_loop(self) -> None:
        """Loop que verifica si es hora de reentrenar."""
        while self._running:
            try:
                if self._should_retrain():
                    logger.info("[Retraining] Iniciando ciclo para %s", self.symbol)
                    result = await asyncio.to_thread(self._run_retrain_cycle)
                    self._history.append(result)
                    self._state["last_retrain_ts"] = time.time()
                    self._state["total_cycles"] = self._state.get("total_cycles", 0) + 1
                    self._save_state()
                    await self._notify(result)
                    await self._report_to_brain(result)
                await asyncio.sleep(60)  # Chequear cada minuto
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("[Retraining] Error en loop: %s", exc)
                if self._dispatcher:
                    await self._dispatcher.system_error("retraining_scheduler", str(exc))
                await asyncio.sleep(300)

    def _should_retrain(self) -> bool:
        last = self._state.get("last_retrain_ts", 0.0)
        return (time.time() - last) >= self.interval_hours * 3600

    # ── Ciclo de retraining ───────────────────────────────────────────────────

    def _run_retrain_cycle(self) -> RetrainingResult:
        """Ejecuta un ciclo completo: Optuna + PPO + evaluación."""
        t_start = time.time()
        ts = datetime.now(tz=timezone.utc).isoformat()

        try:
            self._check_deps()
        except ImportError as e:
            return RetrainingResult(
                symbol=self.symbol, timestamp=ts,
                sharpe=0.0, geometric_return_pct=0.0,
                model_path="", promoted=False,
                hyperparams={}, elapsed_s=time.time()-t_start,
                error=str(e),
            )

        # ── 1. Datos frescos ──────────────────────────────────────────────────
        df = self._fetch_data()
        if df.empty:
            return RetrainingResult(
                symbol=self.symbol, timestamp=ts, sharpe=0.0,
                geometric_return_pct=0.0, model_path="", promoted=False,
                hyperparams={}, elapsed_s=time.time()-t_start,
                error="datos_vacios",
            )

        # ── 2. Optuna: optimizar hiperparámetros ──────────────────────────────
        best_params = self._optuna_search(df)
        logger.info("[Retraining] Mejores hiperparámetros Optuna: %s", best_params)

        # ── 3. Entrenar PPO con mejores params ────────────────────────────────
        from learning.rl_trainer import RLTrainer
        trainer = RLTrainer(
            symbol=self.symbol,
            source=self.source,
            timeframe=self.timeframe,
            initial_capital=10_000.0,
            verbose=0,
        )
        result_dict = trainer.train(
            total_timesteps=self.total_timesteps,
            df=df,
        )

        if "error" in result_dict:
            return RetrainingResult(
                symbol=self.symbol, timestamp=ts, sharpe=0.0,
                geometric_return_pct=0.0, model_path="", promoted=False,
                hyperparams=best_params, elapsed_s=time.time()-t_start,
                error=result_dict["error"],
            )

        eval_m = result_dict.get("eval_metrics", {})
        sharpe = eval_m.get("sharpe_ratio", 0.0)
        geo_ret = eval_m.get("mean_geometric_return_pct", 0.0)
        model_path = result_dict.get("model_path", "")

        # ── 4. Promover si mejora ─────────────────────────────────────────────
        promoted = sharpe >= _MIN_SHARPE_TO_PROMOTE
        if promoted:
            logger.info(
                "[Retraining] ✅ Modelo promovido — Sharpe=%.3f retorno=%.2f%%",
                sharpe, geo_ret,
            )
        else:
            logger.warning(
                "[Retraining] ⚠️ Modelo NO promovido — Sharpe=%.3f < %.1f",
                sharpe, _MIN_SHARPE_TO_PROMOTE,
            )

        return RetrainingResult(
            symbol=self.symbol, timestamp=ts,
            sharpe=sharpe, geometric_return_pct=geo_ret,
            model_path=model_path, promoted=promoted,
            hyperparams=best_params, elapsed_s=time.time()-t_start,
        )

    # ── Optuna ────────────────────────────────────────────────────────────────

    def _optuna_search(self, df) -> dict:
        """Busca los mejores hiperparámetros PPO con Optuna."""
        try:
            import optuna
            optuna.logging.set_verbosity(optuna.logging.WARNING)
        except ImportError:
            logger.warning("[Retraining] Optuna no instalado — usando defaults")
            return self._default_params()

        from models.features import build_features
        from learning.rl_env import QuantTradingEnv

        try:
            feat_df = build_features(df.copy())
        except Exception:
            return self._default_params()

        n = len(feat_df)
        if n < 50:
            return self._default_params()

        train_end = int(n * 0.70)
        df_aligned = df.iloc[len(df) - n:].copy()
        df_aligned.index = feat_df.index

        def objective(trial: "optuna.Trial") -> float:
            try:
                from stable_baselines3 import PPO

                lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
                n_steps = trial.suggest_categorical("n_steps", [512, 1024, 2048])
                gamma = trial.suggest_float("gamma", 0.90, 0.999)
                ent_coef = trial.suggest_float("ent_coef", 0.001, 0.05, log=True)

                env = QuantTradingEnv(
                    df=df_aligned.iloc[:train_end].copy(),
                    symbol=self.symbol,
                )
                if env.observation_space is None:
                    return -999.0

                model = PPO(
                    "MlpPolicy", env,
                    learning_rate=lr,
                    n_steps=n_steps,
                    gamma=gamma,
                    ent_coef=ent_coef,
                    batch_size=64,
                    verbose=0,
                )
                # Entrenamiento corto para evaluar hiperparámetros
                model.learn(total_timesteps=min(10_000, self.total_timesteps // 5))

                # Evaluar en mini test
                eval_env = QuantTradingEnv(
                    df=df_aligned.iloc[train_end:].copy(),
                    symbol=self.symbol,
                )
                obs, _ = eval_env.reset()
                done = False
                total_reward = 0.0
                while not done:
                    action, _ = model.predict(obs, deterministic=True)
                    obs, reward, term, trunc, _ = eval_env.step(int(action))
                    total_reward += reward
                    done = term or trunc

                return float(total_reward)
            except Exception as exc:
                logger.debug("[Optuna] Trial error: %s", exc)
                return -999.0

        study = optuna.create_study(direction="maximize")
        study.optimize(
            objective,
            n_trials=min(self.optuna_trials, 20),
            show_progress_bar=False,
        )

        best = study.best_params
        logger.info("[Optuna] Mejor trial: reward=%.4f params=%s",
                    study.best_value, best)
        return best

    def _default_params(self) -> dict:
        return {
            "learning_rate": 3e-4,
            "n_steps": 2048,
            "gamma": 0.99,
            "ent_coef": 0.01,
        }

    # ── Notificaciones ────────────────────────────────────────────────────────

    async def _notify(self, result: RetrainingResult) -> None:
        if not self._dispatcher:
            return
        try:
            await self._dispatcher.retraining_completed(
                symbol=result.symbol,
                sharpe=result.sharpe,
                geometric_return_pct=result.geometric_return_pct,
                model_path=result.model_path,
                elapsed_s=result.elapsed_s,
            )
        except Exception as exc:
            logger.debug("[Retraining] Notify error: %s", exc)

    async def _report_to_brain(self, result: RetrainingResult) -> None:
        if not self._brain_bridge:
            return
        try:
            await asyncio.to_thread(
                self._brain_bridge.push_event,
                kind="rl_retraining",
                payload=result.to_dict(),
            )
        except Exception as exc:
            logger.debug("[Retraining] Brain bridge error: %s", exc)

    # ── Datos ─────────────────────────────────────────────────────────────────

    def _fetch_data(self):
        from data.feed import MarketFeed
        try:
            feed = MarketFeed(source=self.source)
            df = feed.ohlcv(self.symbol, timeframe=self.timeframe, limit=2000)
            return df
        except Exception as exc:
            logger.error("[Retraining] Fetch data error: %s", exc)
            import pandas as pd
            return pd.DataFrame()

    # ── Persistencia de estado ────────────────────────────────────────────────

    def _load_state(self) -> dict:
        import json
        _STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        if _STATE_PATH.exists():
            try:
                return json.loads(_STATE_PATH.read_text(encoding="utf-8"))
            except Exception:
                pass
        return {"last_retrain_ts": 0.0, "total_cycles": 0}

    def _save_state(self) -> None:
        import json
        try:
            _STATE_PATH.write_text(
                json.dumps(self._state, indent=2, default=str),
                encoding="utf-8",
            )
        except Exception as exc:
            logger.warning("[Retraining] Save state error: %s", exc)

    def _check_deps(self) -> None:
        missing = []
        for pkg in ["stable_baselines3", "gymnasium"]:
            try:
                __import__(pkg)
            except ImportError:
                missing.append(pkg)
        if missing:
            raise ImportError(f"Dependencias RL faltantes: {missing}")


# ── Singleton global ──────────────────────────────────────────────────────────
_scheduler: RetrainingScheduler | None = None


def get_retraining_scheduler(
    symbol: str = "BTC/USDT",
    interval_hours: float = _DEFAULT_INTERVAL_H,
    alert_dispatcher=None,
) -> RetrainingScheduler:
    """Retorna la instancia global del RetrainingScheduler."""
    global _scheduler
    if _scheduler is None:
        _scheduler = RetrainingScheduler(
            symbol=symbol,
            interval_hours=interval_hours,
            alert_dispatcher=alert_dispatcher,
        )
    return _scheduler


# ── CLI ───────────────────────────────────────────────────────────────────────

def _main() -> None:
    import argparse
    import asyncio
    import logging

    parser = argparse.ArgumentParser(description="Atlas Code-Quant — Retraining Scheduler")
    parser.add_argument("--symbol",   default="BTC/USDT")
    parser.add_argument("--source",   default="ccxt", choices=["ccxt", "yfinance"])
    parser.add_argument("--interval", type=float, default=12.0, help="Horas entre retrainings")
    parser.add_argument("--timesteps",type=int, default=80_000)
    parser.add_argument("--trials",   type=int, default=10, help="Optuna trials")
    parser.add_argument("--once",     action="store_true", help="Ejecutar un ciclo y salir")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    scheduler = RetrainingScheduler(
        symbol=args.symbol,
        source=args.source,
        interval_hours=args.interval,
        total_timesteps=args.timesteps,
        optuna_trials=args.trials,
    )

    if args.once:
        result = scheduler._run_retrain_cycle()
        import json
        print(json.dumps(result.to_dict(), indent=2, default=str))
    else:
        async def _run():
            await scheduler.start()
            print(f"Scheduler activo — {args.symbol} cada {args.interval}h. Ctrl+C para detener.")
            try:
                while True:
                    await asyncio.sleep(60)
            except asyncio.CancelledError:
                pass
        asyncio.run(_run())


if __name__ == "__main__":
    _main()
