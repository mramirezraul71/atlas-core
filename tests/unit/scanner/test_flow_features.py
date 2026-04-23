from __future__ import annotations

import pytest

from atlas_scanner.features.flow import (
    normalize_call_put_volume_ratio,
    normalize_oi_change,
    normalize_volume_imbalance,
    resolve_volume_imbalance,
)


def test_normalize_oi_change_respects_config_window() -> None:
    assert normalize_oi_change(0.0, oi_change_min_pct=0.0) == pytest.approx(0.0)
    assert normalize_oi_change(10.0, oi_change_min_pct=0.0) == pytest.approx(100.0)


def test_normalize_call_put_volume_ratio_uses_existing_caps() -> None:
    assert normalize_call_put_volume_ratio(0.7, ratio_min=0.7, ratio_max=2.5) == pytest.approx(0.0)
    assert normalize_call_put_volume_ratio(2.0, ratio_min=0.7, ratio_max=2.5) == pytest.approx(100.0)


def test_volume_imbalance_resolution_and_normalization() -> None:
    direct = resolve_volume_imbalance(volume_imbalance=0.6, call_volume=None, put_volume=None)
    derived = resolve_volume_imbalance(volume_imbalance=None, call_volume=80.0, put_volume=20.0)
    assert direct == pytest.approx(0.6)
    assert derived == pytest.approx(0.6)
    assert normalize_volume_imbalance(-1.0) == pytest.approx(0.0)
    assert normalize_volume_imbalance(1.0) == pytest.approx(100.0)
