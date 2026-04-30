"""
Prophet: breakpoints de tendencia y pico de incertidumbre (Fase 3, opt-in).

Requiere `prophet`. Sin la librería, devuelve señales neutras.
"""
from __future__ import annotations

import logging
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.prophet_detector")


def _to_prophet_frame(close: pd.Series) -> pd.DataFrame:
    idx = close.index
    if isinstance(idx, pd.DatetimeIndex):
        ds = idx
    else:
        ds = pd.date_range("2020-01-01", periods=len(close), freq="D", tz="UTC")
    return pd.DataFrame({"ds": ds, "y": close.astype(float).values})


def fit_prophet_model(close: pd.Series) -> Any | None:
    try:
        from prophet import Prophet  # type: ignore
    except ImportError:
        logger.debug("Prophet no instalado; señales desactivadas")
        return None
    if len(close) < 30:
        return None
    df_p = _to_prophet_frame(close.tail(500))
    model = Prophet(
        changepoint_prior_scale=0.05,
        seasonality_mode="additive",
        yearly_seasonality=False,
        weekly_seasonality=False,
        daily_seasonality=False,
    )
    model.fit(df_p)
    return model


def detect_trend_breakpoint(model: Any, last_n: int = 5) -> tuple[bool, float]:
    if model is None:
        return False, 0.0
    try:
        fut = model.make_future_dataframe(periods=last_n, include_history=True)
        fc = model.predict(fut)
        trend = fc["trend"].values.astype(float)
        if len(trend) < 10:
            return False, 0.0
        recent = trend[-10:-5]
        nxt = trend[-5:]
        if len(recent) < 5 or len(nxt) < 5:
            return False, 0.0
        sr = float(np.polyfit(np.arange(5), recent, 1)[0])
        sn = float(np.polyfit(np.arange(5), nxt, 1)[0])
        denom = abs(sr) + 1e-8
        change = abs(sn - sr) / denom
        return change > 0.3, float(change)
    except Exception as exc:
        logger.debug("detect_trend_breakpoint: %s", exc)
        return False, 0.0


def detect_volatility_spike(model: Any, threshold: float = 1.5) -> tuple[bool, float]:
    if model is None:
        return False, 1.0
    try:
        fut = model.make_future_dataframe(periods=5, include_history=True)
        fc = model.predict(fut)
        u = fc["yhat_upper"].values.astype(float)
        y = fc["yhat"].values.astype(float)
        if len(u) < 10:
            return False, 1.0
        std_recent = float(u[-5] - y[-5])
        std_prev = float(abs(u[-10] - y[-10])) + 1e-12
        ratio = std_recent / std_prev
        return ratio > threshold, float(ratio)
    except Exception as exc:
        logger.debug("detect_volatility_spike: %s", exc)
        return False, 1.0


def prophet_soft_signals(close: pd.Series) -> dict[str, Any]:
    """
    Entrena en la ventana corta y devuelve breakpoint, vol_spike y puntos sugeridos.
    """
    model = fit_prophet_model(close)
    bp, mag = detect_trend_breakpoint(model)
    vol_spike, vol_ratio = detect_volatility_spike(model)
    score_bonus = 0.0
    if not vol_spike:
        score_bonus = 2.0
    return {
        "ok": model is not None,
        "breakpoint": bp,
        "breakpoint_magnitude": mag,
        "volatility_spike": vol_spike,
        "volatility_ratio": vol_ratio,
        "score_bonus_low_vol": score_bonus,
    }
