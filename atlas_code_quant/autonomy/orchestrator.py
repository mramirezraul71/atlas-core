"""Orquestador FSM — F8.

Aplica las transiciones definidas en ``ALLOWED_TRANSITIONS`` y mantiene un
trace_log auditable de los cambios de estado.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from .states import ALLOWED_TRANSITIONS, QuantAutonomyState, is_allowed


@dataclass(slots=True)
class StateTransitionEvent:
    src: QuantAutonomyState
    dst: QuantAutonomyState
    reason: str
    ts: float


@dataclass(slots=True)
class QuantAutonomyOrchestrator:
    state: QuantAutonomyState = QuantAutonomyState.BOOTING
    trace_log: list[StateTransitionEvent] = field(default_factory=list)
    allow_kill_switch_reset: bool = False  # debe activarse explícitamente

    # ── compat F1 ──────────────────────────────────────────────────────────
    def transition_to(
        self,
        new_state: QuantAutonomyState,
        reason: str = "compat_unchecked",
    ) -> QuantAutonomyState:
        """API legacy: NO valida. Mantenida para compat F1."""
        evt = StateTransitionEvent(self.state, new_state, reason, time.time())
        self.trace_log.append(evt)
        self.state = new_state
        return self.state

    # ── F8 ────────────────────────────────────────────────────────────────
    def can_transition(self, dst: QuantAutonomyState) -> bool:
        if (
            self.state == QuantAutonomyState.KILL_SWITCH
            and dst == QuantAutonomyState.BOOTING
            and not self.allow_kill_switch_reset
        ):
            return False
        return is_allowed(self.state, dst)

    def transition(
        self,
        dst: QuantAutonomyState,
        reason: str,
    ) -> StateTransitionEvent:
        """Transición validada. Lanza ValueError si no permitida."""
        if not self.can_transition(dst):
            raise ValueError(
                f"transition_not_allowed: {self.state.value} -> {dst.value}"
            )
        evt = StateTransitionEvent(self.state, dst, reason, time.time())
        self.trace_log.append(evt)
        self.state = dst
        return evt

    def trip_kill_switch(self, reason: str = "manual") -> StateTransitionEvent:
        """Forzar KILL_SWITCH desde cualquier estado (sin validación)."""
        evt = StateTransitionEvent(
            self.state, QuantAutonomyState.KILL_SWITCH, f"trip:{reason}", time.time()
        )
        self.trace_log.append(evt)
        self.state = QuantAutonomyState.KILL_SWITCH
        return evt

    def reachable(self) -> set[QuantAutonomyState]:
        return set(ALLOWED_TRANSITIONS.get(self.state, set()))

    def history(self) -> list[StateTransitionEvent]:
        return list(self.trace_log)
