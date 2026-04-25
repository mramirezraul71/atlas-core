from __future__ import annotations

from atlas_scanner.perception.regulatory.provider import (
    FmpRegulatoryEventProvider,
    StubRegulatoryEventProvider,
    resolve_regulatory_provider,
)


def test_resolve_regulatory_provider_defaults_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_REGULATORY_PROVIDER", raising=False)
    monkeypatch.delenv("ATLAS_FMP_API_KEY", raising=False)
    provider = resolve_regulatory_provider()
    assert isinstance(provider, StubRegulatoryEventProvider)


def test_resolve_regulatory_provider_uses_fmp(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_REGULATORY_PROVIDER", "fmp")
    monkeypatch.setenv("ATLAS_FMP_API_KEY", "token")
    provider = resolve_regulatory_provider()
    assert isinstance(provider, FmpRegulatoryEventProvider)
