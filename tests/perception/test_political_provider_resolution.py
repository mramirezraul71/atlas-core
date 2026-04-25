from __future__ import annotations

from atlas_scanner.perception.political.provider import (
    FinnhubPoliticalTradingProvider,
    StubPoliticalTradingProvider,
    resolve_political_provider,
)


def test_resolve_political_provider_defaults_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_POLITICAL_PROVIDER", raising=False)
    monkeypatch.delenv("ATLAS_FINNHUB_API_KEY", raising=False)
    provider = resolve_political_provider()
    assert isinstance(provider, StubPoliticalTradingProvider)


def test_resolve_political_provider_uses_finnhub_when_key_present(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_POLITICAL_PROVIDER", "finnhub")
    monkeypatch.setenv("ATLAS_FINNHUB_API_KEY", "token")
    provider = resolve_political_provider()
    assert isinstance(provider, FinnhubPoliticalTradingProvider)
