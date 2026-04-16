from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from indicators.stochastic import (  # noqa: E402
    stochastic_divergence_bearish,
    stochastic_divergence_bullish,
    stochastic_signal,
)


def test_stochastic_signal_trend() -> None:
    assert stochastic_signal(15.0, 14.0, 1) is True
    assert stochastic_signal(85.0, 80.0, -1) is True
    assert stochastic_signal(50.0, 50.0, 1) is False


def test_stochastic_divergence_shapes() -> None:
    n = 20
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    close = pd.Series([100.0 - i * 0.5 for i in range(n)], index=idx)
    k = pd.Series([20.0 + i * 0.3 for i in range(n)], index=idx)
    assert stochastic_divergence_bullish(k, close) in {True, False}
    close2 = pd.Series([100.0 + i * 0.5 for i in range(n)], index=idx)
    k2 = pd.Series([80.0 - i * 0.3 for i in range(n)], index=idx)
    assert stochastic_divergence_bearish(k2, close2) in {True, False}
