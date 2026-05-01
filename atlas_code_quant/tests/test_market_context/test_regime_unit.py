from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from market_context.regime_detector import adapt_thresholds_by_regime, detect_regime_from_frame  # noqa: E402


def _df(n: int = 120) -> pd.DataFrame:
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    c = pd.Series([100.0 + i * 0.1 for i in range(n)], index=idx)
    return pd.DataFrame({"open": c - 0.1, "high": c + 0.3, "low": c - 0.3, "close": c, "volume": 1e6}, index=idx)


def test_regime_and_adapt_documented_scales() -> None:
    r = detect_regime_from_frame(_df(120))
    assert r in {"trending_strong", "trending_weak", "consolidating", "high_vol_reversing"}
    a = adapt_thresholds_by_regime("trending_strong")
    assert a["order_flow_weight_scale"] == 1.1
    b = adapt_thresholds_by_regime("consolidating")
    assert b["order_flow_weight_scale"] == 0.9
