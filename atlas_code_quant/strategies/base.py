"""Atlas Code-Quant — Clase base para estrategias event-driven."""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

import pandas as pd

logger = logging.getLogger("quant.strategies")


class Signal(Enum):
    BUY  = "BUY"
    SELL = "SELL"
    HOLD = "HOLD"


@dataclass
class TradeSignal:
    """Señal generada por una estrategia.

    Attributes:
        symbol: Par o ticker (ej: "BTC/USDT").
        signal: BUY / SELL / HOLD.
        confidence: Probabilidad 0.0–1.0.
        price: Precio de referencia en el momento de la señal.
        stop_loss: Precio de stop loss sugerido.
        take_profit: Precio de take profit sugerido.
        metadata: Datos adicionales (indicadores, razones, etc.).
    """
    symbol: str
    signal: Signal
    confidence: float
    price: float
    stop_loss: float | None = None
    take_profit: float | None = None
    timestamp: datetime = field(default_factory=lambda: datetime.now())
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "symbol": self.symbol,
            "signal": self.signal.value,
            "confidence": round(self.confidence, 4),
            "price": self.price,
            "stop_loss": self.stop_loss,
            "take_profit": self.take_profit,
            "timestamp": self.timestamp.isoformat(),
            "metadata": self.metadata,
        }


class BaseStrategy(ABC):
    """Estrategia base event-driven.

    Todas las estrategias heredan de esta clase e implementan `generate_signal()`.

    Args:
        name: Identificador único de la estrategia.
        symbols: Lista de activos que monitorea.
        timeframe: Marco temporal principal.

    Example::
        class MACrossStrategy(BaseStrategy):
            def generate_signal(self, df, symbol):
                fast = df["close"].rolling(10).mean()
                slow = df["close"].rolling(50).mean()
                if fast.iloc[-1] > slow.iloc[-1]:
                    return TradeSignal(symbol, Signal.BUY, 0.7, df["close"].iloc[-1])
                return TradeSignal(symbol, Signal.HOLD, 1.0, df["close"].iloc[-1])
    """

    def __init__(self, name: str, symbols: list[str], timeframe: str = "1h") -> None:
        self.name = name
        self.symbols = symbols
        self.timeframe = timeframe
        self.active = False
        self._signals_history: list[TradeSignal] = []
        logger.info("Estrategia '%s' inicializada para %s", name, symbols)

    @abstractmethod
    def generate_signal(self, df: pd.DataFrame, symbol: str) -> TradeSignal:
        """Procesa OHLCV y retorna una señal de trading.

        Args:
            df: DataFrame con columnas [open, high, low, close, volume], index timestamp.
            symbol: Par/ticker que se está evaluando.

        Returns:
            TradeSignal con la decisión y metadatos.
        """

    def on_signal(self, signal: TradeSignal) -> None:
        """Hook ejecutado cada vez que se genera una señal. Override opcional."""
        self._signals_history.append(signal)
        logger.info("[%s] %s → %s (conf=%.2f)", self.name, signal.symbol,
                    signal.signal.value, signal.confidence)

    def last_signal(self, symbol: str) -> TradeSignal | None:
        """Retorna la última señal generada para un símbolo dado."""
        for s in reversed(self._signals_history):
            if s.symbol == symbol:
                return s
        return None

    def activate(self) -> None:
        self.active = True
        logger.info("Estrategia '%s' ACTIVADA", self.name)

    def deactivate(self) -> None:
        self.active = False
        logger.info("Estrategia '%s' DESACTIVADA", self.name)
