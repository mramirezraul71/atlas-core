from __future__ import annotations

from datetime import datetime, timezone

import atlas_scanner.perception.market.unusual_whales_provider as uw
from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers


class _MockResponse:
    def __init__(self, payload: dict, status_code: int = 200, headers: dict[str, str] | None = None):
        self._payload = payload
        self.status_code = status_code
        self.headers = headers or {}
        self.content = b"x"

    def json(self):
        return self._payload


def test_unusual_whales_maps_flow_alerts_to_raw_events(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    payload = {
        "data": [
            {
                "created_at": "2026-04-25T09:00:00Z",
                "type": "call",
                "ticker": "SPY",
                "strike": "500",
                "expiry": "2026-05-01",
                "total_premium": "250000",
                "total_ask_side_prem": "200000",
                "total_bid_side_prem": "50000",
                "total_size": 120,
                "has_sweep": True,
                "has_floor": False,
            }
        ]
    }

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        _ = args, kwargs
        return _MockResponse(payload, headers={"x-uw-daily-req-count": "1"})

    monkeypatch.setattr(uw.requests, "get", _fake_get)
    config = uw.UnusualWhalesProviderConfig(api_key="demo")
    provider = uw.UnusualWhalesFlowProvider(config=config)
    events = provider.fetch_events(
        symbol="SPY",
        since=datetime.now(timezone.utc),
        until=datetime.now(timezone.utc),
    )
    assert len(events) == 1
    assert events[0].type == "call"
    assert events[0].event_kind == "sweep"
    assert provider.last_diagnostics["status"] == "ok"


def test_unusual_whales_provider_handles_http_error(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        _ = args, kwargs
        return _MockResponse({"error": "denied"}, status_code=401)

    monkeypatch.setattr(uw.requests, "get", _fake_get)
    provider = uw.UnusualWhalesFlowProvider(config=uw.UnusualWhalesProviderConfig(api_key="bad"))
    events = provider.fetch_events(
        symbol="QQQ",
        since=datetime.now(timezone.utc),
        until=datetime.now(timezone.utc),
    )
    assert events == ()
    assert provider.last_diagnostics["status"] == "error"
