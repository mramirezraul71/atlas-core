from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.macro.pipeline import build_macro_context


def test_macro_pipeline_calculates_calendar_risk_and_window() -> None:
    now = datetime.now(timezone.utc)
    context = build_macro_context(
        scope="SPY",
        as_of=now,
        provider_payload={
            "provider_ready": True,
            "confidence": 0.8,
            "upcoming_events": [
                {
                    "event_name": "FOMC Rate Decision",
                    "release_datetime": now + timedelta(hours=1),
                    "impact_level": "high",
                    "affected_sectors": ["financials", "broad_market"],
                }
            ],
            "recent_events": [
                {
                    "event_name": "CPI",
                    "release_datetime": now - timedelta(minutes=30),
                    "surprise": 0.6,
                    "impact_level": "high",
                    "affected_sectors": ["consumer", "broad_market"],
                }
            ],
        },
    )
    assert context.calendar_risk_score > 0.0
    assert context.calendar_volatility_window is True
    assert len(context.upcoming_events) == 1
    assert len(context.recent_events) == 1
