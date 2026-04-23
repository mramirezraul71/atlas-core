from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.data.dummy_gamma_oi import DummyGammaOIProvider
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider
from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider


@dataclass(frozen=True)
class OfflineProviders:
    vol_macro_provider: VolMacroProvider
    gamma_oi_provider: GammaOIProvider


def resolve_offline_providers(
    vol_macro_provider: VolMacroProvider | None = None,
    gamma_oi_provider: GammaOIProvider | None = None,
) -> OfflineProviders:
    return OfflineProviders(
        vol_macro_provider=vol_macro_provider or OpenBBVolMacroProvider(),
        gamma_oi_provider=gamma_oi_provider or DummyGammaOIProvider(),
    )


def is_empty_vol_data(data: VolData) -> bool:
    return not data.iv_history and data.iv_current is None and not data.rv_annualized


def is_empty_macro_data(data: MacroData) -> bool:
    return data.vix is None and data.macro_regime is None and data.seasonal_factor is None


def is_empty_gamma_data(data: GammaData) -> bool:
    return not data.strikes and data.net_gex is None


def is_empty_oi_flow_data(data: OIFlowData) -> bool:
    return (
        data.oi_change_1d_pct is None
        and data.call_put_volume_ratio is None
        and data.volume_imbalance is None
        and data.call_volume is None
        and data.put_volume is None
        and not data.meta
    )

