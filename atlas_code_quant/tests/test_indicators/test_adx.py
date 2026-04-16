from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from indicators.adx import add_adx_columns, get_regime, should_use_breakout, should_use_pullback  # noqa: E402


def _df(n: int = 100) -> pd.DataFrame:
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    c = pd.Series([100.0 + i * 0.2 for i in range(n)], index=idx)
    return pd.DataFrame({"open": c - 0.1, "high": c + 0.3, "low": c - 0.3, "close": c, "volume": 1e6}, index=idx)


def test_adx_columns_and_regime_helpers() -> None:
    df = add_adx_columns(_df(120))
    assert "adx" in df.columns
    adx_last = float(df["adx"].iloc[-1])
    assert get_regime(adx_last) in {"trending_strong", "trending_weak", "consolidating"}
    assert isinstance(should_use_breakout(adx_last), bool)
    assert isinstance(should_use_pullback(adx_last), bool)
