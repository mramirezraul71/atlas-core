from __future__ import annotations

from atlas_scanner.perception.institutional.provider import (
    FmpInstitutionalOwnershipProvider,
    StubInstitutionalOwnershipProvider,
    resolve_institutional_ownership_provider,
)


def test_resolve_ownership_provider_defaults_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_INSTITUTIONAL_PROVIDER", raising=False)
    monkeypatch.delenv("ATLAS_FMP_API_KEY", raising=False)
    provider = resolve_institutional_ownership_provider()
    assert isinstance(provider, StubInstitutionalOwnershipProvider)


def test_resolve_ownership_provider_uses_fmp_when_key(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_INSTITUTIONAL_PROVIDER", "fmp")
    monkeypatch.setenv("ATLAS_FMP_API_KEY", "token")
    provider = resolve_institutional_ownership_provider()
    assert isinstance(provider, FmpInstitutionalOwnershipProvider)
