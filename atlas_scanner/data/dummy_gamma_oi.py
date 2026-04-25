from __future__ import annotations

from dataclasses import dataclass
from datetime import date

from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData


@dataclass
class DummyGammaOIProvider(GammaOIProvider):
    """
    Offline-safe placeholder for gamma/OI provider contract.
    """

    def get_gamma_data(self, symbol: str, as_of: date) -> GammaData:
        _ = (symbol, as_of)
        return GammaData()

    def get_oi_flow_data(self, symbol: str, as_of: date) -> OIFlowData:
        _ = (symbol, as_of)
        return OIFlowData()

