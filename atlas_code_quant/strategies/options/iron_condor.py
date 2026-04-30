"""Scaffold de estrategia iron condor (F1)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class IronCondorStrategy:
    name: str = "iron_condor"

    def build_plan(self, symbol: str) -> dict:
        return {"strategy": self.name, "symbol": symbol, "status": "stub"}
