from __future__ import annotations

from datetime import datetime, timedelta, timezone

from atlas_scanner.perception.common.circuit_breaker import reset_provider_circuit_breakers
from atlas_scanner.perception.institutional.provider import FmpInsiderProviderConfig, FmpInsiderTradingProvider


class _MockResponse:
    def __init__(self, payload, status_code=200) -> None:
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload


def test_fmp_insider_maps_to_snapshot(monkeypatch) -> None:
    reset_provider_circuit_breakers()
    now = datetime.now(timezone.utc)
    payload = [
        {
            "filingDate": (now - timedelta(days=1)).isoformat(),
            "acquistionOrDisposition": "A",
            "securitiesTransacted": "1000",
            "price": "50",
            "reportingName": "John Buyer",
        },
        {
            "filingDate": (now - timedelta(days=2)).isoformat(),
            "acquistionOrDisposition": "D",
            "securitiesTransacted": "200",
            "price": "45",
            "reportingName": "Jane Seller",
        },
    ]

    def _fake_get(*args, **kwargs):  # noqa: ANN002, ANN003
        return _MockResponse(payload)

    monkeypatch.setattr("atlas_scanner.perception.institutional.provider.requests.get", _fake_get)
    provider = FmpInsiderTradingProvider(config=FmpInsiderProviderConfig(api_key="token"))
    snapshot = provider.fetch(symbol="SPY", as_of=now)
    assert snapshot.quality_flags.get("provider_ready") is True
    assert snapshot.buy_sell_ratio is not None and snapshot.buy_sell_ratio > 0
    assert snapshot.meta.get("transaction_count") == 2
    assert snapshot.meta.get("net_signal") == "bullish"
