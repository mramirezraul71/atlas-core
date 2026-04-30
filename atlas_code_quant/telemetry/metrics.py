"""Métricas in-memory para pruebas/smoke F1."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class InMemoryCounter:
    """Counter de métricas sin dependencia externa."""

    values: dict[str, int] = field(default_factory=dict)

    def inc(self, key: str, step: int = 1) -> int:
        self.values[key] = self.values.get(key, 0) + step
        return self.values[key]
