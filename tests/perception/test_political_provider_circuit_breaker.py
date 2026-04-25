from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.political.provider import (
    FinnhubPoliticalProviderConfig,
    FinnhubPoliticalTradingProvider,
)


def test_political_circuit_breaker_opens_after_failures(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    monkeypatch.setenv("ATLAS_PROVIDER_CB_FAILURE_THRESHOLD", "2")
    monkeypatch.setenv("ATLAS_PROVIDER_CB_COOLDOWN_SEC", "60")

    def _boom(*args, **kwargs):  # noqa: ANN002, ANN003
        raise RuntimeError("political_provider_down")

    monkeypatch.setattr("atlas_scanner.perception.political.provider.requests.get", _boom)
    provider = FinnhubPoliticalTradingProvider(config=FinnhubPoliticalProviderConfig(api_key="token"))
    now = datetime.now(timezone.utc)
    provider.fetch(scope="SPY", as_of=now)
    provider.fetch(scope="SPY", as_of=now)
    provider.fetch(scope="SPY", as_of=now)
    assert provider.last_diagnostics.get("error") == "circuit_open"
