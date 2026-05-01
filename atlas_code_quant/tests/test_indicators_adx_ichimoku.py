from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from indicators.adx import add_adx_columns  # noqa: E402
from indicators.ichimoku import add_ichimoku_columns, last_ichimoku_confidence  # noqa: E402


def _ohlcv(n: int = 120) -> pd.DataFrame:
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    close = pd.Series([100.0 + i * 0.05 for i in range(n)], index=idx)
    return pd.DataFrame(
        {
            "open": close - 0.1,
            "high": close + 0.3,
            "low": close - 0.3,
            "close": close,
            "volume": 1e6,
        },
        index=idx,
    )


def test_add_adx_columns_produces_finite_last_row() -> None:
    df = add_adx_columns(_ohlcv(100))
    assert "adx" in df.columns
    assert pd.notna(df["adx"].iloc[-1])


def test_add_ichimoku_and_confidence() -> None:
    df = add_ichimoku_columns(_ohlcv(120))
    assert "tenkan" in df.columns and "senkou_b" in df.columns
    conf = last_ichimoku_confidence(df)
    assert 0.0 <= conf <= 1.0
