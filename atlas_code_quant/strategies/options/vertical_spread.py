"""Scaffold de estrategia vertical spread (F1)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class VerticalSpreadStrategy:
    name: str = "vertical_spread"

    def build_plan(self, symbol: str) -> dict:
        return {"strategy": self.name, "symbol": symbol, "status": "stub"}
