from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.political.provider import (
    FinnhubPoliticalProviderConfig,
    FinnhubPoliticalTradingProvider,
)


class _MockResponse:
    def __init__(self, payload, status_code=200) -> None:
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload


def test_finnhub_political_maps_to_snapshot(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    now = datetime.now(timezone.utc)
    payload = {
        "data": [
            {
                "symbol": "SPY",
                "name": "Senator A",
                "type": "Buy",
                "amount": 25000,
                "transactionDate": (now - timedelta(days=12)).isoformat(),
                "disclosureDate": (now - timedelta(days=4)).isoformat(),
            },
            {
                "symbol": "SPY",
                "name": "Rep B",
                "type": "Sell",
                "amount": 10000,
                "transactionDate": (now - timedelta(days=14)).isoformat(),
                "disclosureDate": (now - timedelta(days=5)).isoformat(),
            },
        ]
    }

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.political.provider.requests.get", _fake_get)
    provider = FinnhubPoliticalTradingProvider(config=FinnhubPoliticalProviderConfig(api_key="token"))
    snapshot = provider.fetch(scope="SPY", as_of=now)
    assert snapshot.quality_flags.get("provider_ready") is True
    assert snapshot.net_political_flow is not None
    assert snapshot.meta.get("transaction_count") == 2
    assert snapshot.meta.get("signal_strength") in {"weak", "medium", "strong"}
    assert snapshot.meta.get("notable_entities")


def test_finnhub_political_degrades_when_missing_fields(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    now = datetime.now(timezone.utc)
    payload = {"data": [{"symbol": "SPY", "name": "Rep C"}]}

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.political.provider.requests.get", _fake_get)
    provider = FinnhubPoliticalTradingProvider(config=FinnhubPoliticalProviderConfig(api_key="token"))
    snapshot = provider.fetch(scope="SPY", as_of=now)
    assert snapshot.quality_flags.get("provider_ready") is False
    assert snapshot.source == "political_fallback_stub"
