from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.features.builders import (
    GammaFeatureInput,
    VolFeatureInput,
    enrich_candidate_with_features,
)
from atlas_scanner.features.gamma import StrikeGamma
from atlas_scanner.models.domain_models import CandidateOpportunity


def _candidate() -> CandidateOpportunity:
    return CandidateOpportunity(
        symbol="SPY",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        asset_type="ETF",
        total_score=0.5,
        explanation="baseline",
    )


def test_enrich_candidate_with_features_complete_inputs() -> None:
    candidate = _candidate()
    vol_input = VolFeatureInput(
        iv_current=40.0,
        iv_history=(10.0, 20.0, 30.0, 40.0, 50.0),
        rv_annualized={"20d": 30.0},
    )
    gamma_input = GammaFeatureInput(
        strike_gamma=(
            StrikeGamma(strike=3900.0, call_gamma=100.0, put_gamma=-20.0),
            StrikeGamma(strike=3950.0, call_gamma=300.0, put_gamma=-40.0),
            StrikeGamma(strike=4000.0, call_gamma=150.0, put_gamma=-400.0),
        ),
        net_gamma=-500_000.0,
    )

    enriched = enrich_candidate_with_features(
        candidate,
        vol_input=vol_input,
        gamma_input=gamma_input,
    )
    assert enriched is not candidate
    assert enriched.vol_features.vrp_20d == 10.0
    assert enriched.gamma_features.gamma_regime == "negative"
    assert enriched.gamma_features.call_wall_nearest is not None
    assert enriched.gamma_features.put_wall_nearest is not None


def test_enrich_candidate_with_features_handles_partial_inputs() -> None:
    candidate = _candidate()

    gamma_only = enrich_candidate_with_features(
        candidate,
        vol_input=None,
        gamma_input=GammaFeatureInput(net_gamma=100.0),
    )
    assert gamma_only.vol_features == candidate.vol_features
    assert gamma_only.gamma_features.gamma_regime == "positive"

    vol_only = enrich_candidate_with_features(
        candidate,
        vol_input=VolFeatureInput(
            iv_current=40.0,
            iv_history=(10.0, 20.0, 30.0, 40.0, 50.0),
            rv_annualized={"5d": 35.0},
        ),
        gamma_input=None,
    )
    assert vol_only.gamma_features == candidate.gamma_features
    assert vol_only.vol_features.vrp_5d == 5.0


def test_enrich_candidate_with_features_with_no_inputs_is_safe() -> None:
    candidate = _candidate()
    enriched = enrich_candidate_with_features(candidate, vol_input=None, gamma_input=None)
    assert enriched == candidate

