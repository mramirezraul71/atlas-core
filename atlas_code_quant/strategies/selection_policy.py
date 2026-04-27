"""Política de selección de estrategias (F1 scaffold)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class SelectionPolicy:
    """Reglas mínimas para filtrar candidatos."""

    min_score: float = 80.0
    max_items: int = 25

    def accepts(self, score: float) -> bool:
        return score >= self.min_score
