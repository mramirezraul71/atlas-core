from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

pytest.importorskip("prophet")

from forecasting.prophet_detector import (  # noqa: E402
    detect_trend_breakpoint,
    detect_volatility_spike,
    fit_prophet_model,
)


def test_fit_prophet_trains() -> None:
    rng = np.random.default_rng(7)
    y = 50 + np.cumsum(rng.normal(0, 0.3, size=120))
    close = pd.Series(y, index=pd.date_range("2023-01-01", periods=120, freq="D", tz="UTC"))
    model = fit_prophet_model(close)
    assert model is not None


def test_breakpoint_detection_slope_change() -> None:
    """Con serie casi lineal, el cambio de pendiente futura suele ser bajo → sin breakpoint."""
    t = np.arange(80, dtype=float)
    y = 0.15 * t + 40.0
    close = pd.Series(y, index=pd.date_range("2023-01-01", periods=80, freq="D", tz="UTC"))
    model = fit_prophet_model(close)
    assert model is not None
    is_bp, mag = detect_trend_breakpoint(model)
    assert isinstance(is_bp, bool)
    assert mag >= 0.0


def test_volatility_spike_returns_ratio() -> None:
    rng = np.random.default_rng(99)
    y = 100 + np.cumsum(rng.normal(0, 1.0, size=100))
    close = pd.Series(y, index=pd.date_range("2022-06-01", periods=100, freq="D", tz="UTC"))
    model = fit_prophet_model(close)
    assert model is not None
    spike, ratio = detect_volatility_spike(model, threshold=10.0)
    assert isinstance(spike, bool)
    assert ratio > 0
