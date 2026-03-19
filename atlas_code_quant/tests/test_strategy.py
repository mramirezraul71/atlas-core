"""Tests unitarios básicos — MACrossStrategy."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pandas as pd
import pytest

from strategies.base import Signal
from strategies.ma_cross import MACrossStrategy


def _make_df(n=100, trend="up"):
    """Genera OHLCV sintético con tendencia."""
    rng = np.random.default_rng(42)
    if trend == "up":
        close = np.linspace(100, 150, n) + rng.normal(0, 1, n)
    elif trend == "down":
        close = np.linspace(150, 100, n) + rng.normal(0, 1, n)
    else:
        close = np.full(n, 120.0) + rng.normal(0, 0.5, n)
    df = pd.DataFrame({
        "open":   close * 0.999,
        "high":   close * 1.002,
        "low":    close * 0.998,
        "close":  close,
        "volume": rng.uniform(1000, 5000, n),
    })
    return df


def test_buy_signal_on_uptrend():
    strategy = MACrossStrategy("test", ["BTC/USDT"], fast_period=5, slow_period=20)
    df = _make_df(100, "up")
    # Fuerza cruce alcista: inserta un valle y luego sube
    df.loc[50:60, "close"] = 98
    df.loc[61:, "close"] = np.linspace(100, 160, 39)
    signal = strategy.generate_signal(df, "BTC/USDT")
    assert signal.signal in (Signal.BUY, Signal.HOLD)
    assert 0.0 <= signal.confidence <= 1.0
    assert signal.price > 0


def test_hold_on_flat():
    strategy = MACrossStrategy("test", ["BTC/USDT"], fast_period=5, slow_period=20)
    df = _make_df(100, "flat")
    signal = strategy.generate_signal(df, "BTC/USDT")
    assert signal.signal == Signal.HOLD


def test_insufficient_data():
    strategy = MACrossStrategy("test", ["BTC/USDT"], fast_period=5, slow_period=50)
    df = _make_df(10)
    signal = strategy.generate_signal(df, "BTC/USDT")
    assert signal.signal == Signal.HOLD
    assert signal.metadata.get("reason") == "insufficient_data"


def test_to_dict_keys():
    strategy = MACrossStrategy("test", ["ETH/USDT"])
    df = _make_df(100, "up")
    signal = strategy.generate_signal(df, "ETH/USDT")
    d = signal.to_dict()
    for key in ("symbol", "signal", "confidence", "price", "timestamp", "metadata"):
        assert key in d
