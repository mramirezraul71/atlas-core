from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.market.unusual_whales_provider import (
    UnusualWhalesFlowProvider,
    UnusualWhalesProviderConfig,
)
from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers


def test_unusual_whales_circuit_breaker_opens_after_failures(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    monkeypatch.setenv("ATLAS_PROVIDER_CB_FAILURE_THRESHOLD", "2")
    monkeypatch.setenv("ATLAS_PROVIDER_CB_COOLDOWN_SEC", "60")

    def _boom(*args, **kwargs):  # noqa: ANN002, ANN003
        raise RuntimeError("network_down")

    monkeypatch.setattr("atlas_scanner.perception.market.unusual_whales_provider.requests.get", _boom)
    provider = UnusualWhalesFlowProvider(config=UnusualWhalesProviderConfig(api_key="token"))
    now = datetime.now(timezone.utc)
    provider.fetch_events(symbol="SPY", since=now - timedelta(hours=1), until=now)
    provider.fetch_events(symbol="SPY", since=now - timedelta(hours=1), until=now)
    provider.fetch_events(symbol="SPY", since=now - timedelta(hours=1), until=now)
    assert provider.last_diagnostics.get("error") == "circuit_open"
