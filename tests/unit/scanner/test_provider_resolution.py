from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.data.dummy_gamma_oi import DummyGammaOIProvider
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider
from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider
from atlas_scanner.runner.provider_resolution import resolve_offline_providers


@dataclass
class _InjectedVolProvider(VolMacroProvider):
    def get_vol_data(self, symbol: str, as_of) -> VolData:
        _ = (symbol, as_of)
        return VolData()

    def get_macro_data(self, as_of) -> MacroData:
        _ = as_of
        return MacroData()


@dataclass
class _InjectedGammaProvider(GammaOIProvider):
    def get_gamma_data(self, symbol: str, as_of) -> GammaData:
        _ = (symbol, as_of)
        return GammaData()

    def get_oi_flow_data(self, symbol: str, as_of) -> OIFlowData:
        _ = (symbol, as_of)
        return OIFlowData()


def test_resolve_offline_providers_uses_defaults() -> None:
    resolved = resolve_offline_providers()
    assert isinstance(resolved.vol_macro_provider, OpenBBVolMacroProvider)
    assert isinstance(resolved.gamma_oi_provider, DummyGammaOIProvider)


def test_resolve_offline_providers_respects_manual_injection() -> None:
    vol_provider = _InjectedVolProvider()
    gamma_provider = _InjectedGammaProvider()
    resolved = resolve_offline_providers(
        vol_macro_provider=vol_provider,
        gamma_oi_provider=gamma_provider,
    )
    assert resolved.vol_macro_provider is vol_provider
    assert resolved.gamma_oi_provider is gamma_provider

