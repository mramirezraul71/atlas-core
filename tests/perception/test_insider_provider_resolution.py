from __future__ import annotations

from atlas_scanner.perception.institutional.provider import (
    FmpInsiderTradingProvider,
    StubInsiderTradingProvider,
    resolve_insider_provider,
)


def test_resolve_insider_provider_defaults_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_INSIDER_PROVIDER", raising=False)
    monkeypatch.delenv("ATLAS_FMP_API_KEY", raising=False)
    provider = resolve_insider_provider()
    assert isinstance(provider, StubInsiderTradingProvider)


def test_resolve_insider_provider_uses_fmp_when_key_present(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_INSIDER_PROVIDER", "fmp")
    monkeypatch.setenv("ATLAS_FMP_API_KEY", "token")
    provider = resolve_insider_provider()
    assert isinstance(provider, FmpInsiderTradingProvider)
