"""Atlas Code-Quant — Entorno de Reinforcement Learning (Gymnasium compatible).

Implementa un entorno de trading para agentes RL siguiendo la interfaz
de Gymnasium (sucesor de OpenAI Gym). Compatible con Stable-Baselines3.

Arquitectura del entorno:
  Estado (obs): Features técnicas logarítmicas (40+) + estado del portfolio (4)
  Acción:       Discrete(3) — 0=HOLD, 1=BUY, 2=SELL
  Recompensa:   Retorno logarítmico del trade - penalización por drawdown/volatilidad
                r = ln(W_t/W_{t-1}) - λ_dd * max(0, dd-dd_target) - λ_vol * realized_vol

Grok/xAI recomendación:
  "RL (PPO) + features híbridas + Kelly risk-constrained = máximos retornos.
   La recompensa es directamente el crecimiento logarítmico de la cartera."

Uso::
    from learning.rl_env import QuantTradingEnv
    env = QuantTradingEnv(df, symbol="BTC/USDT", initial_capital=10_000)
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(1)  # BUY
"""
from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.learning.rl_env")

# ── Acciones ─────────────────────────────────────────────────────────────────
ACTION_HOLD = 0
ACTION_BUY  = 1
ACTION_SELL = 2
N_ACTIONS   = 3

# ── Penalizaciones de recompensa ──────────────────────────────────────────────
_LAMBDA_DRAWDOWN  = 0.5     # Penalización por drawdown excesivo
_LAMBDA_VOL       = 0.1     # Penalización por alta volatilidad
_DD_TARGET        = 0.05    # Drawdown objetivo (5%) — por encima se penaliza
_INVALID_ACTION_P = -0.001  # Penalización por acción inválida (ej: SELL sin posición)


class QuantTradingEnv:
    """Entorno de trading para Reinforcement Learning.

    Compatible con la interfaz Gymnasium (gymnasium.Env).
    NO hereda directamente de gymnasium.Env para evitar dependencia obligatoria
    en producción — pero implementa la misma API (reset/step/render).

    Args:
        df: DataFrame OHLCV con features técnicas precalculadas (build_features()).
            Si no tiene features, se calculan internamente.
        symbol: Ticker para logging.
        initial_capital: Capital inicial en USD.
        commission_pct: Comisión por operación.
        position_size_pct: Porcentaje de capital por posición.
        lambda_drawdown: Peso de penalización por drawdown.
        lambda_vol: Peso de penalización por volatilidad.
        max_steps: Máximo de pasos por episodio (0 = largo completo del df).

    Observation space:
        Box(low=-inf, high=inf, shape=(N_FEATURES + 4,), dtype=float32)
        Los 4 estados del portfolio: [tiene_posicion, pnl_pct, drawdown_pct, log_equity_ratio]

    Action space:
        Discrete(3): 0=HOLD, 1=BUY, 2=SELL
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(
        self,
        df: pd.DataFrame,
        symbol: str = "ASSET",
        initial_capital: float = 10_000.0,
        commission_pct: float = 0.001,
        position_size_pct: float = 0.05,
        lambda_drawdown: float = _LAMBDA_DRAWDOWN,
        lambda_vol: float = _LAMBDA_VOL,
        max_steps: int = 0,
    ) -> None:
        self.symbol = symbol
        self.initial_capital = initial_capital
        self.commission_pct  = commission_pct
        self.position_size_pct = position_size_pct
        self.lambda_dd  = lambda_drawdown
        self.lambda_vol = lambda_vol

        # Preparar features
        self._df_raw = df.copy()
        self._features_df, self._feature_cols = self._prepare_features(df)
        self.n_features = len(self._feature_cols)

        # Observation shape: features + 4 estados del portfolio
        self.obs_shape = (self.n_features + 4,)

        # Gymnasium spaces (definidos como atributos para compatibilidad)
        try:
            import gymnasium as gym
            self.observation_space = gym.spaces.Box(
                low=-np.inf, high=np.inf,
                shape=self.obs_shape, dtype=np.float32,
            )
            self.action_space = gym.spaces.Discrete(N_ACTIONS)
        except ImportError:
            self.observation_space = None  # Funciona sin gymnasium
            self.action_space = None

        self._max_steps = max_steps if max_steps > 0 else len(self._features_df)

        # Estado del episodio (inicializado en reset)
        self._step_idx: int = 0
        self._capital: float = initial_capital
        self._peak_capital: float = initial_capital
        self._position: dict | None = None     # None = sin posición
        self._equity_curve: list[float] = []
        self._log_returns: list[float] = []
        self._total_reward: float = 0.0

    # ── Gymnasium API ─────────────────────────────────────────────────────────

    def reset(
        self,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        """Reinicia el entorno al inicio de un episodio.

        Returns:
            obs: Observación inicial (float32 array).
            info: Dict con métricas iniciales.
        """
        if seed is not None:
            np.random.seed(seed)

        self._step_idx = 0
        self._capital  = self.initial_capital
        self._peak_capital = self.initial_capital
        self._position = None
        self._equity_curve = [self.initial_capital]
        self._log_returns  = []
        self._total_reward = 0.0

        obs = self._get_obs()
        return obs, self._info()

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict]:
        """Ejecuta un paso en el entorno.

        Args:
            action: 0=HOLD, 1=BUY, 2=SELL

        Returns:
            obs: Nueva observación.
            reward: Recompensa logarítmica.
            terminated: True si el capital se agotó o drawdown supera 50%.
            truncated: True si se alcanzó max_steps.
            info: Métricas del paso.
        """
        if self._step_idx >= len(self._features_df) - 1:
            obs = self._get_obs()
            return obs, 0.0, True, False, self._info()

        row = self._features_df.iloc[self._step_idx]
        price = float(self._df_raw.iloc[self._step_idx + len(self._df_raw) - len(self._features_df)]["close"])

        reward = 0.0

        # ── Ejecutar acción ──────────────────────────────────────────────────
        if action == ACTION_BUY:
            if self._position is None:
                reward = self._open_long(price)
            else:
                reward = _INVALID_ACTION_P  # Ya tiene posición

        elif action == ACTION_SELL:
            if self._position is not None:
                reward = self._close_position(price)
            else:
                reward = _INVALID_ACTION_P  # No hay posición

        else:  # HOLD
            # Recompensa por mantener posición ganadora
            if self._position is not None:
                unrealized_log = math.log(price / self._position["entry_price"] + 1e-10)
                reward = unrealized_log * 0.01  # Pequeña recompensa por dirección correcta

        # ── Penalizaciones logarítmicas ──────────────────────────────────────
        current_equity = self._current_equity(price)
        self._equity_curve.append(current_equity)
        self._peak_capital = max(self._peak_capital, current_equity)

        dd = (self._peak_capital - current_equity) / (self._peak_capital + 1e-10)
        dd_penalty = self.lambda_dd * max(0.0, dd - _DD_TARGET)

        vol_penalty = 0.0
        if len(self._log_returns) >= 5:
            realized_vol = float(np.std(self._log_returns[-20:])) * math.sqrt(252)
            vol_penalty = self.lambda_vol * max(0.0, realized_vol - 0.5)  # Penaliza vol > 50% anual

        reward -= (dd_penalty + vol_penalty)
        self._total_reward += reward

        # ── Avanzar paso ────────────────────────────────────────────────────
        self._step_idx += 1

        # ── Condiciones de fin de episodio ───────────────────────────────────
        terminated = (
            current_equity < self.initial_capital * 0.30  # Pérdida >70% capital
            or dd > 0.50                                    # Drawdown >50%
        )
        truncated = self._step_idx >= self._max_steps

        # Cerrar posición al final del episodio
        if (terminated or truncated) and self._position is not None:
            final_price = float(self._df_raw.iloc[
                min(self._step_idx, len(self._df_raw) - 1)
            ]["close"])
            self._close_position(final_price)

        obs = self._get_obs()
        return obs, float(reward), terminated, truncated, self._info()

    def render(self, mode: str = "human") -> str | None:
        """Render básico del estado actual."""
        equity = self._equity_curve[-1] if self._equity_curve else self.initial_capital
        ret_pct = (equity / self.initial_capital - 1) * 100
        pos_str = f"LONG@{self._position['entry_price']:.4f}" if self._position else "FLAT"
        s = (f"[{self.symbol}] Step={self._step_idx} | "
             f"Equity=${equity:.2f} ({ret_pct:+.2f}%) | "
             f"Pos={pos_str} | TotalReward={self._total_reward:.4f}")
        if mode == "human":
            print(s)
        return s

    # ── Métodos internos ──────────────────────────────────────────────────────

    def _open_long(self, price: float) -> float:
        """Abre posición long. Retorna recompensa (0 = sin PnL inmediato)."""
        cost = self._capital * self.position_size_pct
        if cost > self._capital or cost <= 0:
            return _INVALID_ACTION_P
        size = cost / (price * (1 + self.commission_pct))
        self._capital -= size * price * (1 + self.commission_pct)
        self._position = {
            "side": "long",
            "size": size,
            "entry_price": price,
        }
        return 0.0  # Recompensa al cerrar, no al abrir

    def _close_position(self, price: float) -> float:
        """Cierra posición. Retorna el log-return del trade como recompensa."""
        if not self._position:
            return 0.0
        pos = self._position
        proceeds = pos["size"] * price * (1 - self.commission_pct)
        self._capital += proceeds

        log_ret = math.log(price / pos["entry_price"] + 1e-10)
        self._log_returns.append(log_ret)
        self._position = None
        return log_ret  # Recompensa = crecimiento logarítmico del trade

    def _current_equity(self, price: float) -> float:
        eq = self._capital
        if self._position:
            eq += self._position["size"] * price
        return eq

    def _get_obs(self) -> np.ndarray:
        """Construye el vector de observación para el agente RL."""
        idx = min(self._step_idx, len(self._features_df) - 1)
        row = self._features_df.iloc[idx]
        features = row[self._feature_cols].values.astype(np.float32)

        # Estado del portfolio (4 valores normalizados)
        price = float(self._df_raw.iloc[
            max(0, idx + len(self._df_raw) - len(self._features_df))
        ]["close"])
        equity = self._current_equity(price)

        has_position = 1.0 if self._position else 0.0
        pnl_pct = 0.0
        if self._position and self._position["entry_price"] > 0:
            pnl_pct = (price / self._position["entry_price"] - 1.0)

        dd = (self._peak_capital - equity) / (self._peak_capital + 1e-10)
        log_equity_ratio = math.log(equity / self.initial_capital + 1e-10)

        portfolio_state = np.array([
            has_position,
            np.clip(pnl_pct, -1.0, 1.0),
            np.clip(dd, 0.0, 1.0),
            np.clip(log_equity_ratio, -5.0, 5.0),
        ], dtype=np.float32)

        obs = np.concatenate([features, portfolio_state])
        # Reemplazar NaN/Inf con 0
        obs = np.where(np.isfinite(obs), obs, 0.0).astype(np.float32)
        return obs

    def _info(self) -> dict:
        equity = self._equity_curve[-1] if self._equity_curve else self.initial_capital
        return {
            "step":            self._step_idx,
            "equity":          round(equity, 2),
            "total_reward":    round(self._total_reward, 6),
            "log_return":      round(sum(self._log_returns), 6),
            "n_trades":        len(self._log_returns),
            "has_position":    self._position is not None,
        }

    def _prepare_features(self, df: pd.DataFrame) -> tuple[pd.DataFrame, list[str]]:
        """Prepara el DataFrame de features para el entorno RL."""
        from models.features import build_features, get_feature_names

        required_cols = {"open", "high", "low", "close", "volume"}
        if not required_cols.issubset(set(df.columns)):
            logger.error("[RL Env] DataFrame no tiene columnas OHLCV")
            return df, []

        # Si ya tiene features calculadas (más columnas que OHLCV), usarlas
        non_ohlcv = [c for c in df.columns if c not in required_cols and c != "target"]
        if len(non_ohlcv) >= 10:
            # Ya viene con features
            feature_cols = [c for c in non_ohlcv if c != "target"]
            return df[feature_cols + (["target"] if "target" in df.columns else [])].dropna(), feature_cols

        # Calcular features desde OHLCV
        feat_df = build_features(df)
        feat_cols = get_feature_names(feat_df)
        return feat_df, feat_cols

    @property
    def episode_summary(self) -> dict:
        """Retorna resumen del episodio actual."""
        equity = self._equity_curve[-1] if self._equity_curve else self.initial_capital
        log_ret_total = sum(self._log_returns) if self._log_returns else 0.0
        return {
            "symbol":               self.symbol,
            "steps":                self._step_idx,
            "initial_capital":      self.initial_capital,
            "final_equity":         round(equity, 2),
            "total_log_return":     round(log_ret_total, 6),
            "geometric_return_pct": round((math.exp(log_ret_total) - 1) * 100, 2),
            "n_trades":             len(self._log_returns),
            "total_reward":         round(self._total_reward, 6),
        }
