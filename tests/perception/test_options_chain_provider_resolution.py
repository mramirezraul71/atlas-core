from __future__ import annotations

from atlas_scanner.perception.market.options_chain_provider import (
    OpenBbOptionsChainProvider,
    StubOptionsChainProvider,
    resolve_options_chain_provider,
)


def test_options_chain_provider_resolution_stub(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_OPTIONS_CHAIN_PROVIDER", "stub")
    provider = resolve_options_chain_provider()
    assert isinstance(provider, StubOptionsChainProvider)


def test_options_chain_provider_resolution_openbb(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_OPTIONS_CHAIN_PROVIDER", "openbb")
    provider = resolve_options_chain_provider()
    assert isinstance(provider, OpenBbOptionsChainProvider)
