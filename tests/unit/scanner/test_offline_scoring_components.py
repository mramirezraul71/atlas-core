from __future__ import annotations

import pytest

from atlas_scanner.config_loader import (
    ComponentWeights,
    GammaScoringConfig,
    MacroScoringConfig,
    OIFlowScoringConfig,
    PriceScoringConfig,
    VolScoringConfig,
)
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.models.domain_models import GammaFeatures, StrikeLevel, VolFeatures
from atlas_scanner.scoring.offline import (
    ComponentExplanation,
    compute_component_weighted_score,
    compute_gamma_score,
    compute_macro_score,
    compute_oi_flow_score,
    compute_price_score,
    compute_vol_score,
    explain_gamma_component,
    explain_macro_component,
    explain_oi_flow_component,
    explain_price_component,
    explain_vol_component,
    summarize_score_explanation,
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
        oi_flow_score=None,
        price_score=None,
        macro_score=None,
        component_weights=weights,
    )
    only_gamma = compute_component_weighted_score(
        vol_score=None,
        gamma_score=50.0,
        oi_flow_score=None,
        price_score=None,
        macro_score=None,
        component_weights=weights,
    )
    none = compute_component_weighted_score(
        vol_score=None,
        gamma_score=None,
        oi_flow_score=None,
        price_score=None,
        macro_score=None,
        component_weights=weights,
    )
    assert both == pytest.approx(74.0)
    assert only_gamma == pytest.approx(50.0)
    assert none == 0.0


def test_compute_price_score_high_with_favorable_signals() -> None:
    snapshot = SymbolSnapshot(
        symbol="PXH",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.7,
        meta={
            "adx_14": 18.0,
            "trend_state": "RANGING",
            "distance_to_vwap": 0.005,
        },
    )
    score = compute_price_score(snapshot, config=PriceScoringConfig())
    assert score > 75.0


def test_compute_price_score_low_with_unfavorable_signals() -> None:
    snapshot = SymbolSnapshot(
        symbol="PXL",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.7,
        meta={
            "adx_14": 45.0,
            "trend_state": "TREND_UP",
            "distance_to_vwap": 0.08,
        },
    )
    score = compute_price_score(snapshot, config=PriceScoringConfig())
    assert score < 40.0


def test_compute_macro_score_high_for_favorable_context() -> None:
    snapshot = SymbolSnapshot(
        symbol="MCH",
        asset_type="index",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={
            "macro_regime": "risk_on",
            "event_risk": 0.1,
            "vix": 16.0,
            "seasonal_factor": 1.8,
        },
    )
    score = compute_macro_score(snapshot, config=MacroScoringConfig())
    assert score > 70.0


def test_compute_macro_score_low_for_adverse_context() -> None:
    snapshot = SymbolSnapshot(
        symbol="MCL",
        asset_type="index",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={
            "macro_regime": "risk_off",
            "event_risk": 0.95,
            "vix": 35.0,
        },
    )
    score = compute_macro_score(snapshot, config=MacroScoringConfig())
    assert score == 0.0


def test_compute_oi_flow_score_high_for_call_supportive_flow() -> None:
    snapshot = SymbolSnapshot(
        symbol="FLOWH",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={
            "oi_change_1d_pct": 12.0,
            "call_put_volume_ratio": 3.0,
            "volume_imbalance": 0.6,
        },
    )
    score = compute_oi_flow_score(snapshot, config=OIFlowScoringConfig())
    assert score > 75.0


def test_compute_oi_flow_score_low_for_put_defensive_flow() -> None:
    snapshot = SymbolSnapshot(
        symbol="FLOWL",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={
            "oi_change_1d_pct": -5.0,
            "call_put_volume_ratio": 0.4,
            "volume_imbalance": -0.8,
        },
    )
    score = compute_oi_flow_score(snapshot, config=OIFlowScoringConfig())
    assert score < 25.0


def test_compute_oi_flow_score_zero_without_data() -> None:
    snapshot = SymbolSnapshot(
        symbol="FLOW0",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={},
    )
    score = compute_oi_flow_score(snapshot, config=OIFlowScoringConfig())
    assert score == 0.0


def test_explain_vol_component_positive_when_iv_rank_and_vrp_strong() -> None:
    features = VolFeatures(iv_rank_20d=80.0, vrp_20d=8.0)
    explanation = explain_vol_component(vol_features=features, vol_score=85.0, available=True)
    assert explanation.status == "positive"
    assert "IV Rank elevated" in explanation.reasons


def test_explain_gamma_component_negative_for_negative_regime() -> None:
    features = GammaFeatures(gamma_regime="negative")
    explanation = explain_gamma_component(gamma_features=features, gamma_score=20.0, available=True)
    assert explanation.status == "negative"
    assert "negative gamma regime" in explanation.reasons


def test_explain_price_component_ranging_reason() -> None:
    snapshot = SymbolSnapshot(
        symbol="PXR",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={"trend_state": "RANGING", "adx_14": 18.0},
    )
    explanation = explain_price_component(snapshot=snapshot, price_score=70.0, available=True)
    assert explanation.status == "positive"
    assert "range-bound price action" in explanation.reasons


def test_explain_macro_component_flags_elevated_event_risk() -> None:
    snapshot = SymbolSnapshot(
        symbol="MCR",
        asset_type="index",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={"macro_regime": "risk_off", "event_risk": 0.95, "vix": 32.0},
    )
    explanation = explain_macro_component(
        snapshot=snapshot,
        macro_score=0.0,
        available=True,
        config=MacroScoringConfig(),
    )
    assert explanation.status == "negative"
    assert "macro blocked by elevated event risk" in explanation.reasons


def test_explain_oi_flow_component_unavailable_when_missing() -> None:
    snapshot = SymbolSnapshot(
        symbol="NOFLOW",
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={},
    )
    explanation = explain_oi_flow_component(snapshot=snapshot, oi_flow_score=None, available=False)
    assert explanation.status == "unavailable"


def test_summarize_score_explanation_prioritizes_high_score_signals() -> None:
    component_explanations = {
        "vol": ComponentExplanation(
            name="vol",
            score=85.0,
            status="positive",
            reasons=("IV Rank elevated", "positive volatility risk premium"),
        ),
        "gamma": ComponentExplanation(
            name="gamma",
            score=80.0,
            status="positive",
            reasons=("positive gamma regime",),
        ),
        "macro": ComponentExplanation(
            name="macro",
            score=20.0,
            status="negative",
            reasons=("macro regime adverse",),
        ),
    }
    top_reasons = summarize_score_explanation(component_explanations, total_score=0.8)
    assert 2 <= len(top_reasons) <= 4
    assert "IV Rank elevated" in top_reasons

