from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarQualityFlags, RadarSignal
from atlas_scanner.interpretation import interpret_signal


def test_new_dealer_scenarios_are_emitted() -> None:
    signal = RadarSignal(
        symbol="SPY",
        timeframe="5m",
        as_of=datetime.now(timezone.utc),
        direction_score=57.0,
        volume_confirmation_score=60.0,
        flow_conviction_score=63.0,
        dte_pressure_score=56.0,
        dealer_positioning_proxy_score=68.0,
        aggregate_conviction_score=64.0,
        quality=RadarQualityFlags(
            has_market_data=True,
            has_flow_data=True,
            has_greeks_context=True,
            has_oi_context=True,
            provider_quality_ok=True,
            is_degraded=False,
            is_operable=True,
            degradation_reasons=(),
        ),
        primary_conviction_reason="test",
        meta={
            "active_domains": ("market", "flow", "dealer"),
            "horizon_scores": {"intraday": 64.0, "positional": 52.0},
            "dealer_context": {
                "gamma_flip_level": 501.0,
                "call_wall": 505.0,
                "put_wall": 498.0,
                "pinning_zone": (499.0, 503.0),
                "acceleration_zone": (501.0, 506.0),
                "acceleration_direction": "bullish",
                "acceleration_strength": 0.7,
                "pinning_strength": 0.6,
                "gamma_flip_confidence": 0.78,
                "dealer_pressure_score": 60.0,
            },
        },
    )
    names = {scenario.name for scenario in interpret_signal(signal)}
    assert "dealer_gamma_flip_approaching" in names
    assert "dealer_call_wall_active" in names
    assert "dealer_put_wall_active" in names
    assert "dealer_pinning_zone_active" in names
    assert "dealer_acceleration_zone_bullish" in names
