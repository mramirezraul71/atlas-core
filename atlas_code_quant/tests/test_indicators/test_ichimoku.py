from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from indicators.ichimoku import (  # noqa: E402
    add_ichimoku_columns,
    ichimoku_cloud_position,
    ichimoku_signal,
    last_ichimoku_confidence,
)


def _df(n: int = 130) -> pd.DataFrame:
    idx = pd.date_range("2025-01-01", periods=n, freq="D", tz="UTC")
    c = pd.Series([100.0 + i * 0.15 for i in range(n)], index=idx)
    return pd.DataFrame({"open": c - 0.1, "high": c + 0.3, "low": c - 0.3, "close": c, "volume": 1e6}, index=idx)


def test_ichimoku_cloud_position_and_signal() -> None:
    df = add_ichimoku_columns(_df(130))
    row = df.iloc[-1]
    pos = ichimoku_cloud_position(float(row["close"]), float(row["senkou_a"]), float(row["senkou_b"]))
    assert pos in {"above", "inside", "below", None}
    sig = ichimoku_signal(
        float(row["close"]),
        float(row["tenkan"]),
        float(row["kijun"]),
        float(row["senkou_a"]),
        float(row["senkou_b"]),
    )
    assert -10 <= sig <= 10
    conf = last_ichimoku_confidence(df)
    assert 0.0 <= conf <= 1.0
