"""Atlas Code-Quant — Estrategia: Moving Average Crossover.

Señal BUY  cuando MA rápida cruza al alza la MA lenta.
Señal SELL cuando MA rápida cruza a la baja la MA lenta.
Señal HOLD en el resto de casos.
"""
from __future__ import annotations

import pandas as pd

from strategies.base import BaseStrategy, Signal, TradeSignal


class MACrossStrategy(BaseStrategy):
    """Cruce de medias móviles simples (SMA).

    Args:
        name: ID de la estrategia.
        symbols: Lista de activos.
        timeframe: Marco temporal.
        fast_period: Período de la MA rápida (por defecto 10).
        slow_period: Período de la MA lenta (por defecto 50).
        confidence_threshold: Confianza mínima para emitir BUY/SELL (0–1).

    Example::
        strategy = MACrossStrategy("ma_cross", ["BTC/USDT"], fast_period=10, slow_period=50)
        df = feed.ohlcv("BTC/USDT", "1h", limit=200)
        signal = strategy.generate_signal(df, "BTC/USDT")
        print(signal.signal, signal.confidence)
    """

    def __init__(
        self,
        name: str,
        symbols: list[str],
        timeframe: str = "1h",
        fast_period: int = 10,
        slow_period: int = 50,
        confidence_threshold: float = 0.6,
    ) -> None:
        super().__init__(name, symbols, timeframe)
        self.fast_period = fast_period
        self.slow_period = slow_period
        self.confidence_threshold = confidence_threshold

    def generate_signal(self, df: pd.DataFrame, symbol: str) -> TradeSignal:
        """Calcula señal basada en cruce de MAs.

        Args:
            df: OHLCV DataFrame con al menos `slow_period` filas.
            symbol: Par de trading.

        Returns:
            TradeSignal con señal BUY/SELL/HOLD y metadatos de las MAs.
        """
        if len(df) < self.slow_period + 2:
            return TradeSignal(symbol, Signal.HOLD, 1.0, float(df["close"].iloc[-1]),
                               metadata={"reason": "insufficient_data"})

        close = df["close"]
        fast_ma = close.rolling(self.fast_period).mean()
        slow_ma = close.rolling(self.slow_period).mean()

        fast_now  = float(fast_ma.iloc[-1])
        slow_now  = float(slow_ma.iloc[-1])
        fast_prev = float(fast_ma.iloc[-2])
        slow_prev = float(slow_ma.iloc[-2])
        price     = float(close.iloc[-1])

        # Cruce alcista
        if fast_prev <= slow_prev and fast_now > slow_now:
            confidence = min(1.0, abs(fast_now - slow_now) / slow_now * 100)
            confidence = max(0.55, min(0.95, confidence))
            stop  = round(price * 0.98, 6)
            tp    = round(price * 1.04, 6)
            return TradeSignal(
                symbol, Signal.BUY, confidence, price,
                stop_loss=stop, take_profit=tp,
                metadata={"fast_ma": round(fast_now, 4), "slow_ma": round(slow_now, 4),
                          "cross": "golden"},
            )

        # Cruce bajista
        if fast_prev >= slow_prev and fast_now < slow_now:
            confidence = min(1.0, abs(fast_now - slow_now) / slow_now * 100)
            confidence = max(0.55, min(0.95, confidence))
            return TradeSignal(
                symbol, Signal.SELL, confidence, price,
                metadata={"fast_ma": round(fast_now, 4), "slow_ma": round(slow_now, 4),
                          "cross": "death"},
            )

        # Sin cruce — HOLD
        dist_pct = round((fast_now - slow_now) / slow_now * 100, 2)
        return TradeSignal(
            symbol, Signal.HOLD, 1.0, price,
            metadata={"fast_ma": round(fast_now, 4), "slow_ma": round(slow_now, 4),
                      "distance_pct": dist_pct},
        )
