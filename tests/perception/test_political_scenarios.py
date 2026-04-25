from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarQualityFlags, RadarSignal
from atlas_scanner.interpretation import interpret_signal


def test_political_alignment_and_stale_scenarios_are_emitted() -> None:
    signal = RadarSignal(
        symbol="SPY",
        timeframe="1h",
        as_of=datetime.now(timezone.utc),
        direction_score=58.0,
        volume_confirmation_score=60.0,
        flow_conviction_score=56.0,
        dte_pressure_score=52.0,
        dealer_positioning_proxy_score=54.0,
        aggregate_conviction_score=59.0,
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
            "active_domains": ("market", "flow", "political"),
            "freshness": {"swing": {"political": {"status": "stale"}}},
            "political_context": {"net_political_flow": 0.35, "signal_strength": "strong"},
            "horizon_scores": {"intraday": 55.0, "positional": 60.0},
        },
    )
    names = {scenario.name for scenario in interpret_signal(signal)}
    assert "political_interest_context" in names
    assert "political_bullish_alignment" in names
    assert "political_signal_stale" in names
