from __future__ import annotations

from datetime import datetime, timedelta, timezone

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


def test_fmp_ownership_maps_to_snapshot(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    now = datetime.now(timezone.utc)
    payload = [
        {
            "filingDate": (now - timedelta(days=20)).isoformat(),
            "holder": "Fund A",
            "ownershipPercentage": 14.0,
            "changeType": "increase",
        },
        {
            "filingDate": (now - timedelta(days=22)).isoformat(),
            "holder": "Fund B",
            "ownershipPercentage": 9.0,
            "changeType": "new",
        },
    ]

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.institutional.provider.requests.get", _fake_get)
    provider = FmpInstitutionalOwnershipProvider(config=FmpOwnershipProviderConfig(api_key="token"))
    snapshot = provider.fetch(symbol="SPY", as_of=now)
    assert snapshot.quality_flags.get("provider_ready") is True
    assert snapshot.concentration_score is not None
    assert snapshot.meta.get("sponsorship_score") is not None
    assert snapshot.meta.get("ownership_signal") in {"bullish", "neutral", "bearish"}
