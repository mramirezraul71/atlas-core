from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.macro.calendar_provider import TradingEconomicsCalendarProvider


class _MockResponse:
    def __init__(self, payload, status_code=200) -> None:
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload


def test_recent_events_degrades_when_actual_consensus_missing(monkeypatch) -> None:
    now = datetime.now(timezone.utc)
    payload = [
        {
            "Event": "CPI",
            "Date": (now - timedelta(hours=1)).isoformat(),
            "Importance": "3",
            "Previous": "3.1",
        }
    ]

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.macro.calendar_provider.requests.get", _fake_get)
    provider = TradingEconomicsCalendarProvider(api_key="token")
    events = provider.fetch_recent_events(
        from_date=now - timedelta(hours=24),
        to_date=now + timedelta(hours=2),
    )
    assert len(events) == 1
    assert events[0].surprise is None
