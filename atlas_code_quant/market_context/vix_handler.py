"""VIX: lectura vía yfinance, multiplicador de riesgo y gate operativo."""
from __future__ import annotations

import math
from typing import Any

import pandas as pd


def fetch_vix() -> float | None:
    """Último cierre del VIX (^VIX) o None si no hay datos."""
    s = _fetch_vix_history(10)
    if s is None or s.empty:
        return None
    return float(s.iloc[-1])


def calculate_vix_zscore(vix_current: float, vix_ma20: float, vix_std20: float) -> float:
    if vix_std20 <= 0 or not math.isfinite(vix_std20):
        return 0.0
    return (vix_current - vix_ma20) / vix_std20


def get_vix_multiplier_from_zscore(z: float) -> float:
    """Multiplicador 0.5–1.0 a partir del z-score del VIX."""
    if z > 1.5:
        return 0.5
    if z > 0.8:
        return 0.7
    return 1.0


def _fetch_vix_history(days: int = 120) -> pd.Series | None:
    try:
        import yfinance as yf  # type: ignore

        t = yf.Ticker("^VIX")
        h = t.history(period=f"{max(days, 30)}d", interval="1d", auto_adjust=True)
        if h is None or h.empty or "Close" not in h.columns:
            return None
        return h["Close"].astype(float).dropna()
    except Exception:
        return None


def get_vix_zscore_multiplier(vix_current: float, vix_ma20: float, vix_std20: float) -> float:
    if vix_std20 <= 0 or not math.isfinite(vix_std20):
        return 1.0
    z = calculate_vix_zscore(vix_current, vix_ma20, vix_std20)
    return get_vix_multiplier_from_zscore(z)


def get_vix_gate(vix_current: float) -> str:
    if vix_current > 35:
        return "panic"
    if vix_current > 30:
        return "caution"
    return "normal"


def build_vix_context() -> dict[str, Any]:
    """Contexto VIX para un ciclo del escáner."""
    series = _fetch_vix_history()
    if series is None or len(series) < 5:
        return {
            "current": None,
            "ma20": None,
            "std20": None,
            "zscore": None,
            "multiplier": 1.0,
            "gate": "normal",
            "ok": False,
            "reason": "vix_unavailable",
        }
    current = float(series.iloc[-1])
    tail = series.tail(20)
    ma20 = float(tail.mean())
    std20 = float(tail.std(ddof=0)) if len(tail) > 1 else 0.0
    zscore = calculate_vix_zscore(current, ma20, std20)
    mult = get_vix_zscore_multiplier(current, ma20, std20)
    gate = get_vix_gate(current)
    return {
        "current": round(current, 3),
        "ma20": round(ma20, 3),
        "std20": round(std20, 4),
        "zscore": round(zscore, 3),
        "multiplier": mult,
        "gate": gate,
        "ok": True,
        "reason": "ok",
    }
