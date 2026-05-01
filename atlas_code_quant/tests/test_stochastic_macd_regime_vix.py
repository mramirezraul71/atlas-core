from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from indicators.macd import add_macd_columns, macd_divergence  # noqa: E402
from indicators.stochastic import add_stochastic_columns, stochastic_pullback_bull  # noqa: E402
from market_context.regime_detector import adapt_thresholds_by_regime, detect_regime_from_frame  # noqa: E402
from market_context.vix_handler import get_vix_gate, get_vix_zscore_multiplier  # noqa: E402


def _df(n: int = 200) -> pd.DataFrame:
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    c = pd.Series([100.0 + i * 0.1 for i in range(n)], index=idx)
    return pd.DataFrame({"open": c - 0.1, "high": c + 0.2, "low": c - 0.2, "close": c, "volume": 1e6}, index=idx)


def test_stochastic_columns_and_pullback_bull() -> None:
    df = add_stochastic_columns(_df(100))
    assert "stoch_k" in df.columns
    assert stochastic_pullback_bull(15.0, 22.0, 20.0)


def test_macd_divergence_none_on_flat() -> None:
    df = add_macd_columns(_df(80))
    assert macd_divergence(df["close"], df["macd"]) is None


def test_regime_detector_returns_label() -> None:
    r = detect_regime_from_frame(_df(120))
    assert r in {"trending_strong", "trending_weak", "consolidating", "high_vol_reversing"}
    a = adapt_thresholds_by_regime(r)
    assert "min_win_rate_delta" in a and "order_flow_weight_scale" in a
    assert adapt_thresholds_by_regime("trending_strong")["order_flow_weight_scale"] == 1.1
    assert adapt_thresholds_by_regime("consolidating")["order_flow_weight_scale"] == 0.9


def test_vix_gate_and_multiplier() -> None:
    assert get_vix_gate(36.0) == "panic"
    assert get_vix_gate(31.0) == "caution"
    assert get_vix_gate(18.0) == "normal"
    assert get_vix_zscore_multiplier(30.0, 20.0, 5.0) < 1.0
