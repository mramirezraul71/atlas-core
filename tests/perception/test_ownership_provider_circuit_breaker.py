from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.institutional.provider import (
    FmpInstitutionalOwnershipProvider,
    FmpOwnershipProviderConfig,
)


def test_ownership_circuit_breaker_opens_after_failures(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    monkeypatch.setenv("ATLAS_PROVIDER_CB_FAILURE_THRESHOLD", "2")
    monkeypatch.setenv("ATLAS_PROVIDER_CB_COOLDOWN_SEC", "60")

    def _boom(*args, **kwargs):  # noqa: ANN002, ANN003
        raise RuntimeError("ownership_provider_down")

    monkeypatch.setattr("atlas_scanner.perception.institutional.provider.requests.get", _boom)
    provider = FmpInstitutionalOwnershipProvider(config=FmpOwnershipProviderConfig(api_key="token"))
    now = datetime.now(timezone.utc)
    provider.fetch(symbol="SPY", as_of=now)
    provider.fetch(symbol="SPY", as_of=now)
    provider.fetch(symbol="SPY", as_of=now)
    assert provider.last_diagnostics.get("error") == "circuit_open"
