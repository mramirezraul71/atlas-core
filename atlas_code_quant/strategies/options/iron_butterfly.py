"""Adapter conservador de Iron Butterfly para F1.

Reutiliza la implementación existente en `atlas_code_quant.iron_butterfly`
sin eliminar el módulo legacy original.
"""
from __future__ import annotations

from dataclasses import dataclass

from atlas_code_quant.iron_butterfly import IronButterflyBacktester


@dataclass(slots=True)
class IronButterflyStrategy:
    name: str = "iron_butterfly"

    def create_backtester(self) -> IronButterflyBacktester:
        return IronButterflyBacktester()

    def build_plan(self, symbol: str) -> dict:
        return {"strategy": self.name, "symbol": symbol, "status": "adapter"}
