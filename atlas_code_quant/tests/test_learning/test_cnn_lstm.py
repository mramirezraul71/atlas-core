from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from learning.cnn_lstm_pattern import (  # noqa: E402
    pattern_boost_points,
    predict_pattern_confidence,
    prepare_candlestick_image,
)


def _sample_ohlcv(n: int = 120) -> pd.DataFrame:
    rng = np.random.default_rng(42)
    close = 100 + np.cumsum(rng.normal(0, 0.5, size=n))
    high = close + rng.uniform(0.1, 0.8, size=n)
    low = close - rng.uniform(0.1, 0.8, size=n)
    open_ = np.roll(close, 1)
    open_[0] = close[0]
    vol = rng.integers(1_000, 50_000, size=n).astype(float)
    idx = pd.date_range("2020-01-01", periods=n, freq="h", tz="UTC")
    return pd.DataFrame({"open": open_, "high": high, "low": low, "close": close, "volume": vol}, index=idx)


def test_prepare_image_shape() -> None:
    df = _sample_ohlcv()
    img = prepare_candlestick_image(df, lookback=20, img_size=28)
    assert img.shape == (28, 28, 6)


def test_model_build_summary() -> None:
    tf = pytest.importorskip("tensorflow")
    from learning.cnn_lstm_pattern import build_cnn_lstm_model

    m = build_cnn_lstm_model()
    s = []
    m.summary(print_fn=lambda line: s.append(line))
    text = "\n".join(s)
    assert "conv2d" in text.lower() or "Conv2D" in text
    del m
    tf.keras.backend.clear_session()


def test_predict_confidence_uniform_without_model() -> None:
    img = np.zeros((28, 28, 6), dtype=np.float32)
    pred = predict_pattern_confidence(img, None)
    assert set(pred.keys()) == {"no_pattern", "bullish_pattern", "bearish_pattern"}
    assert abs(sum(pred.values()) - 1.0) < 1e-5


def test_pattern_boost_scoring_long() -> None:
    pred = {"no_pattern": 0.1, "bullish_pattern": 0.8, "bearish_pattern": 0.1}
    pts = pattern_boost_points(pred, direction=1, threshold=0.65)
    assert pts == pytest.approx((0.8 - 0.5) * 10.0)
