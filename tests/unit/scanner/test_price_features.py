from __future__ import annotations

import pytest

from atlas_scanner.features.price import (
    interpret_trend_state,
    normalize_adx,
    normalize_distance_to_vwap,
)


def test_normalize_adx_prefers_ranging_regime() -> None:
    assert normalize_adx(18.0, ranging_max=20.0, trending_min=35.0) == pytest.approx(100.0)
    assert normalize_adx(35.0, ranging_max=20.0, trending_min=35.0) == pytest.approx(0.0)


def test_interpret_trend_state_maps_expected_labels() -> None:
    assert interpret_trend_state("RANGING") == 80.0
    assert interpret_trend_state("trend_up") == 60.0
    assert interpret_trend_state("UNKNOWN") == 50.0
    assert interpret_trend_state("SIDEWAYS") is None


def test_normalize_distance_to_vwap_rewards_proximity() -> None:
    assert normalize_distance_to_vwap(0.0) == pytest.approx(100.0)
    assert normalize_distance_to_vwap(0.05) == pytest.approx(0.0)
    assert normalize_distance_to_vwap(None) is None
