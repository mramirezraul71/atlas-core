"""Régimen de mercado heurístico (sin HMM) a partir de ADX y ATR."""
from __future__ import annotations

import math
from typing import Any

import pandas as pd

from indicators.adx import add_adx_columns


def _atr_series(df: pd.DataFrame, period: int = 14) -> pd.Series:
    high = df["high"].astype(float)
    low = df["low"].astype(float)
    close = df["close"].astype(float)
    prev_close = close.shift(1)
    tr = pd.concat(
        [
            (high - low).abs(),
            (high - prev_close).abs(),
            (low - prev_close).abs(),
        ],
        axis=1,
    ).max(axis=1)
    return tr.rolling(period, min_periods=period).mean()


def detect_regime_from_frame(df: pd.DataFrame) -> str:
    """Estados: trending_strong | trending_weak | consolidating | high_vol_reversing."""
    if df is None or len(df) < 60:
        return "high_vol_reversing"
    work = add_adx_columns(df.copy(), period=14) if "adx" not in df.columns else df
    adx_val = float(work["adx"].iloc[-1]) if "adx" in work.columns and pd.notna(work["adx"].iloc[-1]) else 20.0
    atr = _atr_series(work, 14)
    atr_now = float(atr.iloc[-1]) if pd.notna(atr.iloc[-1]) else 0.0
    atr_ma = float(atr.tail(60).mean()) if len(atr.dropna()) else atr_now
    ratio = atr_now / atr_ma if atr_ma > 0 else 1.0

    if not math.isfinite(adx_val):
        adx_val = 20.0
    if adx_val > 25 and ratio > 1.5:
        return "trending_strong"
    if adx_val > 20:
        return "trending_weak"
    if adx_val < 15 and ratio < 0.8:
        return "consolidating"
    return "high_vol_reversing"


def adapt_thresholds_by_regime(regime: str) -> dict[str, float]:
    """Delta en puntos % sobre min win rate local; escala order flow (multiplicador)."""
    if regime == "trending_strong":
        return {"min_win_rate_delta": -2.0, "order_flow_weight_scale": 1.1}
    if regime == "consolidating":
        return {"min_win_rate_delta": 2.0, "order_flow_weight_scale": 0.9}
    if regime == "trending_weak":
        return {"min_win_rate_delta": 0.0, "order_flow_weight_scale": 1.0}
    return {"min_win_rate_delta": 0.0, "order_flow_weight_scale": 1.0}


def regime_dict(regime: str, adapt: dict[str, float]) -> dict[str, Any]:
    return {"label": regime, **adapt}
