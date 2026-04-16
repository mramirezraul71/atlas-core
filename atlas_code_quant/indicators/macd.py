"""MACD y detección simple de divergencia vs precio."""
from __future__ import annotations

from typing import Literal

import pandas as pd


def macd(
    close: pd.Series,
    fast: int = 12,
    slow: int = 26,
    signal_period: int = 9,
) -> tuple[pd.Series, pd.Series, pd.Series]:
    ema_fast = close.ewm(span=fast, adjust=False).mean()
    ema_slow = close.ewm(span=slow, adjust=False).mean()
    macd_line = ema_fast - ema_slow
    signal_line = macd_line.ewm(span=signal_period, adjust=False).mean()
    histogram = macd_line - signal_line
    return macd_line, signal_line, histogram


def add_macd_columns(
    df: pd.DataFrame,
    fast: int = 12,
    slow: int = 26,
    signal_period: int = 9,
) -> pd.DataFrame:
    if "close" not in df.columns:
        return df
    out = df.copy()
    mline, sig, hist = macd(out["close"].astype(float), fast, slow, signal_period)
    out["macd"] = mline
    out["macd_signal"] = sig
    out["macd_hist"] = hist
    return out


def macd_divergence(
    price_series: pd.Series,
    macd_line: pd.Series,
    lookback: int = 5,
) -> Literal["bearish_divergence", "bullish_divergence"] | None:
    """Última barra: precio vs MACD en ventana corta."""
    if len(price_series) < lookback + 1 or len(macd_line) < lookback + 1:
        return None
    p0 = float(price_series.iloc[-1])
    p1 = float(price_series.iloc[-lookback])
    m0 = float(macd_line.iloc[-1])
    m1 = float(macd_line.iloc[-lookback])
    price_change = p0 - p1
    macd_change = m0 - m1
    if price_change > 0 and macd_change < 0:
        return "bearish_divergence"
    if price_change < 0 and macd_change > 0:
        return "bullish_divergence"
    return None
