from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.institutional.provider import (
    FmpInstitutionalOwnershipProvider,
    FmpOwnershipProviderConfig,
)


class _MockResponse:
    def __init__(self, payload, status_code=200) -> None:
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload


def test_ownership_provider_degrades_when_missing_fields(monkeypatch) -> None:
    reset_provider_circuit_breakers()

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse([{"holder": "Fund A"}])

    monkeypatch.setattr("atlas_scanner.perception.institutional.provider.requests.get", _fake_get)
    provider = FmpInstitutionalOwnershipProvider(config=FmpOwnershipProviderConfig(api_key="token"))
    snapshot = provider.fetch(symbol="SPY", as_of=datetime.now(timezone.utc))
    assert snapshot.quality_flags.get("provider_ready") is False
    assert snapshot.source == "ownership_fallback_stub"
