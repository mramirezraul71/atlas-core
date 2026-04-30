"""Oscilador estocástico (%K / %D) para pullbacks en tendencia."""
from __future__ import annotations

import pandas as pd


def stochastic(
    high: pd.Series,
    low: pd.Series,
    close: pd.Series,
    period: int = 14,
    smooth_k: int = 3,
    smooth_d: int = 3,
) -> tuple[pd.Series, pd.Series]:
    """%K suavizado y %D (señal)."""
    lowest_low = low.rolling(window=period, min_periods=period).min()
    highest_high = high.rolling(window=period, min_periods=period).max()
    rng = (highest_high - lowest_low).replace(0, pd.NA)
    k_raw = 100.0 * ((close.astype(float) - lowest_low.astype(float)) / rng)
    k_percent = k_raw.rolling(window=smooth_k, min_periods=smooth_k).mean()
    d_percent = k_percent.rolling(window=smooth_d, min_periods=smooth_d).mean()
    return k_percent.fillna(50.0), d_percent.fillna(50.0)


def add_stochastic_columns(
    df: pd.DataFrame,
    period: int = 14,
    smooth_k: int = 3,
    smooth_d: int = 3,
) -> pd.DataFrame:
    if not {"high", "low", "close"}.issubset(df.columns):
        return df
    out = df.copy()
    k, d = stochastic(out["high"], out["low"], out["close"], period, smooth_k, smooth_d)
    out["stoch_k"] = k
    out["stoch_d"] = d
    return out


def stochastic_pullback_bull(k_prev: float, k_now: float, d_now: float) -> bool:
    """Cruce desde sobreventa: salida de zona <20 con impulso al alza."""
    return k_prev < 20.0 and k_now >= 20.0 and k_now >= k_prev


def stochastic_pullback_bear(k_prev: float, k_now: float, d_now: float) -> bool:
    """Cruce desde sobrecompra: caída desde >80."""
    return k_prev > 80.0 and k_now <= 80.0 and k_now <= k_prev


def stochastic_signal(k: float, d: float, trend_direction: int) -> bool:
    """Condición de pullback según tendencia (resumen Fase 2)."""
    if trend_direction == 1:
        return k < 20.0
    if trend_direction == -1:
        return k > 80.0
    return False


def stochastic_divergence_bullish(k_series: pd.Series, close: pd.Series, last_n: int = 5) -> bool:
    """Precio hace mínimo más bajo pero K no confirma (posible giro alcista)."""
    if len(k_series) < last_n + 1 or len(close) < last_n + 1:
        return False
    c0, c1 = float(close.iloc[-1]), float(close.iloc[-last_n])
    k0, k1 = float(k_series.iloc[-1]), float(k_series.iloc[-last_n])
    return c0 < c1 and k0 > k1


def stochastic_divergence_bearish(k_series: pd.Series, close: pd.Series, last_n: int = 5) -> bool:
    """Precio hace máximo más alto pero K no confirma (posible giro bajista)."""
    if len(k_series) < last_n + 1 or len(close) < last_n + 1:
        return False
    c0, c1 = float(close.iloc[-1]), float(close.iloc[-last_n])
    k0, k1 = float(k_series.iloc[-1]), float(k_series.iloc[-last_n])
    return c0 > c1 and k0 < k1
