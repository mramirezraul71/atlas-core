from __future__ import annotations

import pytest

from atlas_scanner.config_loader import (
    ComponentWeights,
    GammaScoringConfig,
    MacroScoringConfig,
    OIFlowScoringConfig,
    OfflineScoringConfig,
    PriceScoringConfig,
    VolScoringConfig,
)


def test_offline_scoring_config_default_construction() -> None:
    config = OfflineScoringConfig()

    assert isinstance(config.component_weights, ComponentWeights)
    assert isinstance(config.vol, VolScoringConfig)
    assert isinstance(config.gamma, GammaScoringConfig)
    assert isinstance(config.oi_flow, OIFlowScoringConfig)
    assert isinstance(config.price, PriceScoringConfig)
    assert isinstance(config.macro, MacroScoringConfig)


def test_component_weights_sum_is_reasonable() -> None:
    weights = OfflineScoringConfig().component_weights
    total = weights.vol + weights.gamma + weights.oi_flow + weights.price + weights.macro
    assert total == pytest.approx(1.0)


def test_component_weights_can_be_customized() -> None:
    custom = OfflineScoringConfig(
        component_weights=ComponentWeights(
            vol=0.50,
            gamma=0.20,
            oi_flow=0.10,
            price=0.15,
            macro=0.05,
        )
    )
    assert custom.component_weights.vol == 0.50
    assert custom.component_weights.price == 0.15


def test_component_thresholds_are_accessible() -> None:
    config = OfflineScoringConfig()
    assert config.vol.iv_rank_min == 40.0
    assert config.gamma.max_pain_distance_pct_max == 1.5
    assert config.oi_flow.call_put_volume_ratio_max == 999.0
    assert config.price.adx_ranging_max == 25.0
    assert config.macro.block_on_event_risk_high is True


def test_component_weights_normalized_helper() -> None:
    weights = ComponentWeights(
        vol=2.0,
        gamma=1.0,
        oi_flow=1.0,
        price=0.0,
        macro=0.0,
    )
    normalized = weights.normalized()
    total = (
        normalized.vol
        + normalized.gamma
        + normalized.oi_flow
        + normalized.price
        + normalized.macro
    )
    assert total == pytest.approx(1.0)
    assert normalized.vol == pytest.approx(0.5)
    assert normalized.gamma == pytest.approx(0.25)
    assert normalized.oi_flow == pytest.approx(0.25)

