from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarQualityFlags, RadarSignal
from atlas_scanner.interpretation import interpret_signal


def test_macro_calendar_scenarios_are_emitted() -> None:
    signal = RadarSignal(
        symbol="SPY",
        timeframe="5m",
        as_of=datetime.now(timezone.utc),
        direction_score=60.0,
        volume_confirmation_score=62.0,
        flow_conviction_score=64.0,
        dte_pressure_score=55.0,
        dealer_positioning_proxy_score=58.0,
        aggregate_conviction_score=61.0,
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
            "active_domains": ("market", "flow", "macro"),
            "horizon_scores": {"intraday": 62.0, "positional": 50.0},
            "macro_calendar": {
                "calendar_risk_score": 0.8,
                "calendar_volatility_window": True,
                "recent_surprise": -0.3,
            },
        },
    )
    scenarios = interpret_signal(signal)
    names = {scenario.name for scenario in scenarios}
    assert "macro_event_imminent" in names
    assert "macro_volatility_window_active" in names
    assert "macro_surprise_bearish" in names
