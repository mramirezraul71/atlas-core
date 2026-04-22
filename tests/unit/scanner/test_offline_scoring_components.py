from __future__ import annotations

import pytest

from atlas_scanner.config_loader import ComponentWeights, GammaScoringConfig, VolScoringConfig
from atlas_scanner.models.domain_models import GammaFeatures, StrikeLevel, VolFeatures
from atlas_scanner.scoring.offline import (
    compute_component_weighted_score,
    compute_gamma_score,
    compute_vol_score,
)


def test_compute_vol_score_high_when_iv_rank_and_vrp_are_strong() -> None:
    features = VolFeatures(
        iv_rank_20d=80.0,
        vrp_20d=10.0,
    )
    score = compute_vol_score(features, config=VolScoringConfig(iv_rank_min=40.0, iv_rank_max=85.0, vrp_min=0.0))
    assert score > 70.0


def test_compute_vol_score_low_when_iv_rank_below_threshold() -> None:
    features = VolFeatures(
        iv_rank_20d=10.0,
        vrp_20d=-5.0,
    )
    score = compute_vol_score(features, config=VolScoringConfig(iv_rank_min=40.0, iv_rank_max=85.0, vrp_min=0.0))
    assert score == 0.0


def test_compute_vol_score_returns_zero_when_features_missing() -> None:
    assert compute_vol_score(None, config=VolScoringConfig()) == 0.0
    assert compute_vol_score(VolFeatures(), config=VolScoringConfig()) == 0.0


def test_compute_gamma_score_high_for_positive_complete_structure() -> None:
    features = GammaFeatures(
        gamma_regime="positive",
        gamma_flip_agg=5100.0,
        call_wall_nearest=StrikeLevel(strike=5200.0, kind="CALL_WALL"),
        put_wall_nearest=StrikeLevel(strike=5000.0, kind="PUT_WALL"),
        net_gex=-100000.0,
    )
    score = compute_gamma_score(features, config=GammaScoringConfig())
    assert score == 100.0


def test_compute_gamma_score_low_for_negative_regime() -> None:
    features = GammaFeatures(gamma_regime="negative")
    score = compute_gamma_score(features, config=GammaScoringConfig())
    assert score == 20.0


def test_compute_gamma_score_returns_zero_when_features_missing() -> None:
    assert compute_gamma_score(None, config=GammaScoringConfig()) == 0.0
    assert compute_gamma_score(GammaFeatures(), config=GammaScoringConfig()) == 0.0


def test_compute_component_weighted_score_renormalizes_available_components() -> None:
    weights = ComponentWeights(vol=0.6, gamma=0.4, oi_flow=0.0, price=0.0, macro=0.0)
    both = compute_component_weighted_score(
        vol_score=90.0,
        gamma_score=50.0,
        component_weights=weights,
    )
    only_gamma = compute_component_weighted_score(
        vol_score=None,
        gamma_score=50.0,
        component_weights=weights,
    )
    none = compute_component_weighted_score(
        vol_score=None,
        gamma_score=None,
        component_weights=weights,
    )
    assert both == pytest.approx(74.0)
    assert only_gamma == pytest.approx(50.0)
    assert none == 0.0

