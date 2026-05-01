"""Atlas Code-Quant — Estrategia basada en agente RL (PPO).

Wrapper que adapta un modelo PPO entrenado (Stable-Baselines3) a la
interfaz BaseStrategy del sistema Atlas Code-Quant. Permite usar el
agente RL exactamente igual que MACrossStrategy o MLSignalStrategy:
con generate_signal(df, symbol) → TradeSignal.

El agente decide en cada barra si comprar, vender o esperar basándose
en el estado del portfolio y las features técnicas logarítmicas.

Uso::
    # Con modelo ya entrenado:
    strategy = RLStrategy.from_model_path(
        path="models/saved/ppo_BTC_USDT_20260321.zip",
        symbols=["BTC/USDT"],
    )
    signal = strategy.generate_signal(df, "BTC/USDT")

    # Auto-carga el modelo más reciente para el símbolo:
    strategy = RLStrategy.load_latest("BTC/USDT")
"""
from __future__ import annotations

import logging
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from strategies.base import BaseStrategy, Signal, TradeSignal

logger = logging.getLogger("quant.strategies.rl")

# ── Constantes ────────────────────────────────────────────────────────────────
ACTION_HOLD = 0
ACTION_BUY  = 1
ACTION_SELL = 2
_MODELS_DIR = Path(__file__).resolve().parent.parent / "models" / "saved"


class RLStrategy(BaseStrategy):
    """Estrategia de trading basada en agente RL (PPO) entrenado.

    Convierte OHLCV → features → estado → acción RL → TradeSignal.

    Args:
        model: Modelo PPO cargado (stable_baselines3.PPO).
        symbols: Lista de activos que esta estrategia monitorea.
        timeframe: Marco temporal.
        initial_capital: Capital para calcular el estado del portfolio.
        confidence_threshold: Confianza mínima para emitir señal BUY/SELL (0-1).
        deterministic: Si True, el agente toma la acción más probable (sin exploración).

    Attributes:
        name: "rl_ppo" (identificador en el journal y selector).
    """

    def __init__(
        self,
        model: Any,
        symbols: list[str],
        timeframe: str = "1h",
        initial_capital: float = 10_000.0,
        confidence_threshold: float = 0.55,
        deterministic: bool = True,
    ) -> None:
        super().__init__(
            name="rl_ppo",
            symbols=symbols,
            timeframe=timeframe,
        )
        self._model = model
        self._capital = initial_capital
        self._initial_capital = initial_capital
        self._confidence_threshold = confidence_threshold
        self._deterministic = deterministic
        # Estado del portfolio por símbolo
        self._positions: dict[str, dict | None] = {s: None for s in symbols}
        self._peak_capital: float = initial_capital

    # ── BaseStrategy interface ────────────────────────────────────────────────

    def generate_signal(self, df: pd.DataFrame, symbol: str) -> TradeSignal:
        """Genera una señal de trading usando el agente RL.

        Args:
            df: OHLCV DataFrame con al menos 50 barras (para features).
            symbol: Ticker evaluado.

        Returns:
            TradeSignal con BUY/SELL/HOLD y confianza estimada.
        """
        price = float(df["close"].iloc[-1])

        if len(df) < 50:
            return TradeSignal(
                symbol=symbol, signal=Signal.HOLD,
                confidence=0.0, price=price,
                metadata={"reason": "datos_insuficientes"},
            )

        try:
            obs = self._build_obs(df, symbol)
        except Exception as exc:
            logger.warning("[RLStrategy] Error construyendo observación para %s: %s", symbol, exc)
            return TradeSignal(
                symbol=symbol, signal=Signal.HOLD,
                confidence=0.0, price=price,
                metadata={"reason": f"obs_error: {exc}"},
            )

        # Predicción del agente PPO
        action, extras = self._model.predict(obs, deterministic=self._deterministic)
        action = int(action)

        # Estimar confianza desde la distribución de acciones (si disponible)
        confidence = self._estimate_confidence(obs)

        # Calcular ATR para stops dinámicos
        atr = self._compute_atr(df)
        sl, tp = self._atr_stops(price, atr, side="long")

        # Mapear acción → Signal (con validación de estado del portfolio)
        position = self._positions.get(symbol)
        signal = Signal.HOLD

        if action == ACTION_BUY and position is None and confidence >= self._confidence_threshold:
            signal = Signal.BUY
        elif action == ACTION_SELL and position is not None:
            signal = Signal.SELL
        # HOLD en cualquier otro caso

        trade_signal = TradeSignal(
            symbol=symbol,
            signal=signal,
            confidence=round(confidence, 4),
            price=price,
            stop_loss=sl if signal == Signal.BUY else None,
            take_profit=tp if signal == Signal.BUY else None,
            metadata={
                "strategy":     "rl_ppo",
                "action_raw":   action,
                "atr":          round(atr, 4),
                "has_position": position is not None,
            },
        )

        # Actualizar estado del portfolio simulado
        self._update_position_state(symbol, signal, price)

        return trade_signal

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_obs(self, df: pd.DataFrame, symbol: str) -> np.ndarray:
        """Construye el vector de observación igual que QuantTradingEnv._get_obs()."""
        from models.features import build_features, get_feature_names

        feat_df = build_features(df.copy())
        if feat_df.empty:
            raise ValueError("Features vacías")

        feature_cols = get_feature_names(feat_df)
        features = feat_df[feature_cols].iloc[-1].values.astype(np.float32)

        # Estado del portfolio (mismo orden que QuantTradingEnv)
        pos = self._positions.get(symbol)
        has_position = 1.0 if pos else 0.0
        pnl_pct = 0.0
        if pos and pos.get("entry_price", 0) > 0:
            pnl_pct = (float(df["close"].iloc[-1]) / pos["entry_price"] - 1.0)

        equity = self._capital
        if pos:
            equity += pos.get("size", 0) * float(df["close"].iloc[-1])
        self._peak_capital = max(self._peak_capital, equity)

        dd = (self._peak_capital - equity) / (self._peak_capital + 1e-10)
        log_equity_ratio = math.log(equity / self._initial_capital + 1e-10)

        portfolio_state = np.array([
            has_position,
            np.clip(pnl_pct, -1.0, 1.0),
            np.clip(dd, 0.0, 1.0),
            np.clip(log_equity_ratio, -5.0, 5.0),
        ], dtype=np.float32)

        obs = np.concatenate([features, portfolio_state])
        return np.where(np.isfinite(obs), obs, 0.0).astype(np.float32)

    def _estimate_confidence(self, obs: np.ndarray) -> float:
        """Estima confianza usando la distribución de probabilidades del policy."""
        try:
            import torch
            obs_tensor = self._model.policy.obs_to_tensor(obs)[0]
            with torch.no_grad():
                dist = self._model.policy.get_distribution(obs_tensor)
                probs = dist.distribution.probs.cpu().numpy().flatten()
            return float(probs.max())
        except Exception:
            return 0.60  # Default razonable si no puede calcular

    def _compute_atr(self, df: pd.DataFrame, period: int = 14) -> float:
        """Calcula ATR de las últimas barras."""
        try:
            from models.features import compute_atr
            atr_series = compute_atr(df, period)
            atr = float(atr_series.iloc[-1])
            return atr if math.isfinite(atr) and atr > 0 else float(df["close"].iloc[-1]) * 0.01
        except Exception:
            return float(df["close"].iloc[-1]) * 0.01

    def _atr_stops(self, price: float, atr: float, side: str = "long") -> tuple[float, float]:
        """ATR stops: SL=1.5×ATR, TP=3.0×ATR (criterio Kelly/Grok)."""
        if side == "long":
            return (price - 1.5 * atr, price + 3.0 * atr)
        return (price + 1.5 * atr, price - 3.0 * atr)

    def _update_position_state(self, symbol: str, signal: Signal, price: float) -> None:
        """Mantiene el estado de posición simulado para el cálculo de observaciones."""
        if signal == Signal.BUY and self._positions.get(symbol) is None:
            size = (self._capital * 0.05) / price
            self._positions[symbol] = {"entry_price": price, "size": size}
        elif signal == Signal.SELL and self._positions.get(symbol) is not None:
            pos = self._positions[symbol]
            pnl = pos["size"] * (price - pos["entry_price"])
            self._capital += pnl
            self._positions[symbol] = None

    # ── Factory methods ───────────────────────────────────────────────────────

    @classmethod
    def from_model_path(
        cls,
        path: str,
        symbols: list[str],
        timeframe: str = "1h",
        **kwargs,
    ) -> "RLStrategy":
        """Carga una RLStrategy desde la ruta de un modelo guardado."""
        try:
            from stable_baselines3 import PPO
        except ImportError:
            raise ImportError("stable-baselines3 no instalado — pip install stable-baselines3")
        model = PPO.load(path)
        logger.info("[RLStrategy] Modelo cargado: %s", path)
        return cls(model=model, symbols=symbols, timeframe=timeframe, **kwargs)

    @classmethod
    def load_latest(
        cls,
        symbol: str,
        timeframe: str = "1h",
        **kwargs,
    ) -> "RLStrategy | None":
        """Carga el modelo PPO más reciente para el símbolo dado."""
        safe_sym = symbol.replace("/", "_")
        matches = sorted(_MODELS_DIR.glob(f"ppo_{safe_sym}_*.zip"), reverse=True)
        if not matches:
            logger.warning("[RLStrategy] No se encontró modelo PPO para %s en %s", symbol, _MODELS_DIR)
            return None
        latest = matches[0]
        return cls.from_model_path(str(latest), symbols=[symbol], timeframe=timeframe, **kwargs)

    @classmethod
    def is_model_available(cls, symbol: str) -> bool:
        """Retorna True si existe un modelo PPO entrenado para el símbolo."""
        safe_sym = symbol.replace("/", "_")
        return any(_MODELS_DIR.glob(f"ppo_{safe_sym}_*.zip"))
