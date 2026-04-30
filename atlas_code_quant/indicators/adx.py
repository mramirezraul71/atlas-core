"""ADX (+DI / −DI) sobre OHLCV (Wilder / EWM compatible con ta-lib aprox.)."""
from __future__ import annotations

import math

import pandas as pd


def add_adx_columns(df: pd.DataFrame, period: int = 14) -> pd.DataFrame:
    """Añade columnas `plus_di`, `minus_di`, `adx` al DataFrame."""
    need = {"high", "low", "close"}
    if not need.issubset(df.columns):
        return df
    out = df.copy()
    high = out["high"].astype(float)
    low = out["low"].astype(float)
    close = out["close"].astype(float)
    prev_close = close.shift(1)

    tr = pd.concat(
        [
            (high - low).abs(),
            (high - prev_close).abs(),
            (low - prev_close).abs(),
        ],
        axis=1,
    ).max(axis=1)

    up_move = high.diff()
    down_move = -low.diff()
    plus_dm = ((up_move > down_move) & (up_move > 0.0)) * up_move
    minus_dm = ((down_move > up_move) & (down_move > 0.0)) * down_move

    alpha = 1.0 / float(period)
    atr = tr.ewm(alpha=alpha, adjust=False, min_periods=period).mean()
    plus_dm_s = plus_dm.ewm(alpha=alpha, adjust=False, min_periods=period).mean()
    minus_dm_s = minus_dm.ewm(alpha=alpha, adjust=False, min_periods=period).mean()

    plus_di = 100.0 * (plus_dm_s / atr.replace(0, pd.NA))
    minus_di = 100.0 * (minus_dm_s / atr.replace(0, pd.NA))
    denom = (plus_di + minus_di).replace(0, pd.NA)
    dx = 100.0 * (plus_di - minus_di).abs() / denom
    adx = dx.ewm(alpha=alpha, adjust=False, min_periods=period).mean()

    out["plus_di"] = plus_di
    out["minus_di"] = minus_di
    out["adx"] = adx
    return out


def get_regime(adx_value: float) -> str:
    """Clasificación simple por ADX (último valor típico 0–100)."""
    try:
        v = float(adx_value)
    except (TypeError, ValueError):
        return "consolidating"
    if not math.isfinite(v):
        return "consolidating"
    if v > 25:
        return "trending_strong"
    if v > 20:
        return "trending_weak"
    return "consolidating"


def should_use_breakout(adx_value: float) -> bool:
    """Breakouts / Donchian tienen más sentido con tendencia clara."""
    return get_regime(adx_value) == "trending_strong"


def should_use_pullback(adx_value: float) -> bool:
    """Pullbacks / mean-reversion condicionada mejor en rango o tendencia débil."""
    return get_regime(adx_value) in {"consolidating", "trending_weak"}
