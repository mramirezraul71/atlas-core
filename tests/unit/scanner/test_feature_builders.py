from __future__ import annotations

from atlas_scanner.features.builders import build_gamma_features, build_vol_features
from atlas_scanner.features.gamma import StrikeGamma
from atlas_scanner.models.domain_models import GammaFeatures, VolFeatures


def test_build_vol_features_complete_input() -> None:
    result = build_vol_features(
        iv_current=40.0,
        iv_history=(10.0, 20.0, 30.0, 40.0, 50.0),
        rv_annualized={"5d": 20.0, "10d": 25.0, "20d": 30.0, "60d": 35.0},
    )

    assert isinstance(result, VolFeatures)
    assert result.iv_rank_20d == 75.0
    assert result.iv_percentile == 80.0
    assert result.vrp_5d == 20.0
    assert result.vrp_10d == 15.0
    assert result.vrp_20d == 10.0
    assert result.vrp_60d == 5.0


def test_build_vol_features_missing_iv_history_is_safe() -> None:
    result = build_vol_features(
        iv_current=30.0,
        iv_history=None,
        rv_annualized={"5d": 20.0},
    )

    assert result.iv_rank_20d is None
    assert result.iv_percentile is None
    assert result.vrp_5d == 10.0


def test_build_vol_features_missing_rv_is_safe() -> None:
    result = build_vol_features(
        iv_current=30.0,
        iv_history=(10.0, 20.0, 30.0),
        rv_annualized=None,
    )

    assert result.iv_rank_20d is not None
    assert result.vrp_5d is None
    assert result.vrp_10d is None
    assert result.vrp_20d is None
    assert result.vrp_60d is None


def test_build_gamma_features_complete_input() -> None:
    strikes = (
        StrikeGamma(strike=3900.0, call_gamma=120.0, put_gamma=-80.0),
        StrikeGamma(strike=3950.0, call_gamma=300.0, put_gamma=-120.0),
        StrikeGamma(strike=4000.0, call_gamma=180.0, put_gamma=-250.0),
    )
    result = build_gamma_features(
        strike_gamma=strikes,
        net_gamma=-500_000.0,
        neutral_threshold=10_000.0,
    )

    assert isinstance(result, GammaFeatures)
    assert result.call_wall_nearest is not None
    assert result.call_wall_nearest.strike == 3950.0
    assert result.put_wall_nearest is not None
    assert result.put_wall_nearest.strike == 4000.0
    assert result.gamma_flip_agg is not None
    assert result.gamma_regime == "negative"
    assert result.net_gex == -500_000.0


def test_build_gamma_features_missing_strikes_is_safe() -> None:
    result = build_gamma_features(
        strike_gamma=None,
        net_gamma=500.0,
    )
    assert result.call_wall_nearest is None
    assert result.put_wall_nearest is None
    assert result.gamma_flip_agg is None
    assert result.gamma_regime == "positive"


def test_build_gamma_features_missing_net_gamma_is_safe() -> None:
    result = build_gamma_features(
        strike_gamma=(),
        net_gamma=None,
    )
    assert result.net_gex is None
    assert result.gamma_regime is None

