"""Tests unitarios — Feature engineering."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pandas as pd
import pytest

from models.features import build_features, get_feature_names


def _make_df(n: int = 300) -> pd.DataFrame:
    rng = np.random.default_rng(1)
    close = np.linspace(100, 150, n) + rng.normal(0, 1, n)
    return pd.DataFrame(
        {
            "open":   close * 0.999,
            "high":   close * 1.005,
            "low":    close * 0.995,
            "close":  close,
            "volume": rng.uniform(1000, 5000, n),
        },
        index=pd.date_range("2024-01-01", periods=n, freq="1h"),
    )


def test_build_features_returns_dataframe():
    df = _make_df(300)
    out = build_features(df, target_bars=5)
    assert isinstance(out, pd.DataFrame)
    assert len(out) > 0


def test_target_column_exists():
    df = _make_df(300)
    out = build_features(df, target_bars=5)
    assert "target" in out.columns


def test_target_values_valid():
    df = _make_df(300)
    out = build_features(df, target_bars=5)
    assert set(out["target"].unique()).issubset({-1, 0, 1})


def test_no_nan_in_features():
    df = _make_df(300)
    out = build_features(df, target_bars=5)
    assert out.isnull().sum().sum() == 0


def test_feature_names_excludes_target():
    df = _make_df(300)
    out = build_features(df, target_bars=5)
    names = get_feature_names(out)
    assert "target" not in names
    assert len(names) > 10
