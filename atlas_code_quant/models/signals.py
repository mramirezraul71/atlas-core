"""Atlas Code-Quant — Estrategia ML-driven.

Usa un modelo entrenado para generar señales de trading,
integrándola con la interfaz BaseStrategy.
"""
from __future__ import annotations

import logging

import pandas as pd

from strategies.base import BaseStrategy, Signal, TradeSignal

logger = logging.getLogger("quant.ml_strategy")


class MLSignalStrategy(BaseStrategy):
    """Estrategia basada en modelo de ML (RandomForest / LightGBM).

    Args:
        name: ID de la estrategia.
        symbols: Lista de activos.
        model_name: "rf" | "gb" | "lgbm"
        confidence_threshold: Confianza mínima para emitir señal (0–1).
        target_bars: Ventana futura del target con que fue entrenado.

    Example::
        strategy = MLSignalStrategy("ml_rf", ["BTC/USDT"], model_name="rf")
        signal = strategy.generate_signal(df, "BTC/USDT")
    """

    def __init__(
        self,
        name: str,
        symbols: list[str],
        timeframe: str = "1h",
        model_name: str = "rf",
        confidence_threshold: float = 0.60,
        target_bars: int = 5,
    ) -> None:
        super().__init__(name, symbols, timeframe)
        self.model_name           = model_name
        self.confidence_threshold = confidence_threshold
        self.target_bars          = target_bars
        self._pipeline            = None
        self._feature_names: list[str] = []
        self._load_model()

    def _load_model(self) -> None:
        try:
            from models.trainer import load_model
            self._pipeline, self._feature_names = load_model(self.model_name)
            logger.info("Modelo ML cargado: %s (%d features)", self.model_name, len(self._feature_names))
        except FileNotFoundError:
            logger.warning("Modelo '%s' no entrenado aún. Usando HOLD.", self.model_name)

    def generate_signal(self, df: pd.DataFrame, symbol: str) -> TradeSignal:
        price = float(df["close"].iloc[-1])

        if self._pipeline is None:
            return TradeSignal(
                symbol, Signal.HOLD, 1.0, price,
                metadata={"reason": "model_not_trained"},
            )

        from models.trainer import predict_signal
        result = predict_signal(
            self._pipeline,
            self._feature_names,
            df,
            target_bars=self.target_bars,
        )

        raw_signal  = result["signal"]
        confidence  = result["confidence"]
        proba_dict  = result["probabilities"]

        if confidence < self.confidence_threshold:
            return TradeSignal(
                symbol, Signal.HOLD, confidence, price,
                metadata={"reason": "low_confidence", "confidence": confidence, "proba": proba_dict},
            )

        if raw_signal == 1:
            stop = round(price * 0.98, 6)
            tp   = round(price * 1.04, 6)
            return TradeSignal(
                symbol, Signal.BUY, confidence, price,
                stop_loss=stop, take_profit=tp,
                metadata={"model": self.model_name, "proba": proba_dict},
            )

        if raw_signal == -1:
            return TradeSignal(
                symbol, Signal.SELL, confidence, price,
                metadata={"model": self.model_name, "proba": proba_dict},
            )

        return TradeSignal(
            symbol, Signal.HOLD, confidence, price,
            metadata={"model": self.model_name, "proba": proba_dict},
        )
