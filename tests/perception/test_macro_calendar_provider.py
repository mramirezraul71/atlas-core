from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.macro.calendar_provider import (
    StubCalendarProvider,
    calendar_window_hours,
    resolve_calendar_provider,
)


def test_stub_calendar_provider_returns_events() -> None:
    now = datetime.now(timezone.utc)
    provider = StubCalendarProvider()
    events = provider.fetch_calendar(from_date=now - timedelta(hours=1), to_date=now + timedelta(hours=72))
    assert len(events) >= 1
    assert any(event.impact_level == "high" for event in events)


def test_calendar_resolver_defaults_to_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_ECONOMIC_CALENDAR_PROVIDER", raising=False)
    provider = resolve_calendar_provider()
    assert provider.__class__.__name__ == "StubCalendarProvider"


def test_calendar_window_hours_defaults(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_ECONOMIC_CALENDAR_LOOKAHEAD_HOURS", raising=False)
    monkeypatch.delenv("ATLAS_ECONOMIC_CALENDAR_LOOKBACK_HOURS", raising=False)
    lookahead, lookback = calendar_window_hours()
    assert lookahead == 72
    assert lookback == 24
