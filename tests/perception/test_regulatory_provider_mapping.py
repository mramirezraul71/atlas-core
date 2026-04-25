from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.regulatory.provider import FmpRegulatoryEventProvider, FmpRegulatoryProviderConfig


class _MockResponse:
    def __init__(self, payload, status_code=200) -> None:
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload


def test_fmp_regulatory_maps_to_snapshot(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    now = datetime.now(timezone.utc)
    payload = [
        {
            "fillingDate": (now - timedelta(days=1)).isoformat(),
            "formType": "8-K",
            "acceptedDate": (now - timedelta(days=1)).isoformat(),
        },
        {
            "fillingDate": (now - timedelta(days=2)).isoformat(),
            "formType": "10-Q",
            "acceptedDate": (now - timedelta(days=2)).isoformat(),
        },
    ]

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.regulatory.provider.requests.get", _fake_get)
    provider = FmpRegulatoryEventProvider(config=FmpRegulatoryProviderConfig(api_key="token"))
    snapshot = provider.fetch(symbol_or_scope="SPY", as_of=now)
    assert snapshot.quality_flags.get("provider_ready") is True
    assert snapshot.overhang_score is not None and snapshot.overhang_score > 0
    assert snapshot.meta.get("filing_count") == 2
