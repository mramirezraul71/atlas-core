"""Orquestador autónomo mínimo (F1 scaffold)."""
from __future__ import annotations

from dataclasses import dataclass

from .states import QuantAutonomyState


@dataclass(slots=True)
class QuantAutonomyOrchestrator:
    """Mantiene estado básico sin activar loops en F1."""

    state: QuantAutonomyState = QuantAutonomyState.BOOTING

    def transition_to(self, new_state: QuantAutonomyState) -> QuantAutonomyState:
        self.state = new_state
        return self.state
