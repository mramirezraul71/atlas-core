"""Scaffold combinado straddle/strangle (F1)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class StraddleStrangleStrategy:
    name: str = "straddle_strangle"

    def build_plan(self, symbol: str, mode: str = "straddle") -> dict:
        return {
            "strategy": self.name,
            "symbol": symbol,
            "mode": mode,
            "status": "stub",
        }
