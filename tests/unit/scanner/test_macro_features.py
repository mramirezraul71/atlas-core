from __future__ import annotations

import pytest

from atlas_scanner.features.macro import (
    classify_macro_regime_from_vix,
    normalize_event_risk,
    normalize_seasonal_factor,
    score_macro_regime,
    score_vix_bucket,
)


def test_classify_macro_regime_from_vix_uses_expected_buckets() -> None:
    assert classify_macro_regime_from_vix(18.0) == "favorable"
    assert classify_macro_regime_from_vix(25.0) == "neutral"
    assert classify_macro_regime_from_vix(35.0) == "adverse"


def test_score_macro_regime_supports_existing_labels() -> None:
    assert score_macro_regime("risk_on") == 80.0
    assert score_macro_regime("neutral") == 50.0
    assert score_macro_regime("risk_off") == 20.0
    assert score_macro_regime("unknown") is None


def test_macro_numeric_normalizers_keep_contract() -> None:
    assert normalize_event_risk(0.0) == pytest.approx(100.0)
    assert normalize_event_risk(1.0) == pytest.approx(0.0)
    assert score_vix_bucket(16.0) == 80.0
    assert score_vix_bucket(24.0) == 55.0
    assert score_vix_bucket(35.0) == 30.0
    assert normalize_seasonal_factor(1.8, 0.8, 1.8) == pytest.approx(100.0)
