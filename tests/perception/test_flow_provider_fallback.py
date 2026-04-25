from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.market.flow_provider import (
    NoOpFlowEventsProvider,
    SyntheticFlowEventsProvider,
    resolve_flow_provider,
)


def test_flow_provider_resolves_synthetic_by_default() -> None:
    resolution = resolve_flow_provider(use_real_provider=False)
    assert resolution.provider_name == "synthetic"
    assert isinstance(resolution.provider, SyntheticFlowEventsProvider)
    events = resolution.provider.fetch_events(
        symbol="SPY",
        since=datetime.now(timezone.utc) - timedelta(minutes=5),
        until=datetime.now(timezone.utc),
    )
    assert len(events) > 0


def test_flow_provider_real_mode_falls_back_to_noop_stub() -> None:
    resolution = resolve_flow_provider(use_real_provider=True, provider_name="noop")
    assert isinstance(resolution.provider, NoOpFlowEventsProvider)
    events = resolution.provider.fetch_events(
        symbol="SPY",
        since=datetime.now(timezone.utc) - timedelta(minutes=5),
        until=datetime.now(timezone.utc),
    )
    assert events == ()


def test_flow_provider_unusual_whales_falls_back_without_key(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_UNUSUAL_WHALES_API_KEY", raising=False)
    resolution = resolve_flow_provider(provider_name="unusual_whales", fallback_name="synthetic")
    assert resolution.fallback_used is True
    assert resolution.provider_name == "synthetic"
    assert resolution.reason == "unusual_whales_missing_api_key"
