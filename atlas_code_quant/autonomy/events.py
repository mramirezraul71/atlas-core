"""Atlas Code Quant — Autonomy events (F17).

Eventos que el orquestador autónomo consume en cada ``step``.

F17 sólo necesita un conjunto mínimo de eventos para impulsar la FSM
en modo paper:

* ``Tick``: pulso periódico (sin payload obligatorio).
* ``OpportunityArrived``: nueva oportunidad Radar aprobada.
* ``StrategyBuilt``: factory generó intents para una oportunidad.
* ``FitnessReady``: F14 produjo fitness para los intents.
* ``PaperExecuted``: F16 envió las órdenes paper (dry-run).
* ``DegradationDetected`` / ``RecoveryDetected``: cambios en gates de
  salud/Radar.
* ``KillSwitchTriggered``: kill switch activado (F19 lo conectará a
  fichero y límites).
* ``LiveArmRequested``: petición externa de armado live. F17 la
  registra pero **NO transiciona a ``LIVE_ARMED``**.

Diseño defensivo: los eventos son ``dataclass(frozen=True)`` con
campos opcionales. El orquestador nunca asume que el payload está
completo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class AutonomyEvent:
    """Base para todos los eventos del orquestador."""

    name: str
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Tick(AutonomyEvent):
    name: str = "tick"


@dataclass(frozen=True)
class OpportunityArrived(AutonomyEvent):
    name: str = "opportunity_arrived"


@dataclass(frozen=True)
class StrategyBuilt(AutonomyEvent):
    name: str = "strategy_built"


@dataclass(frozen=True)
class FitnessReady(AutonomyEvent):
    name: str = "fitness_ready"


@dataclass(frozen=True)
class PaperExecuted(AutonomyEvent):
    name: str = "paper_executed"


@dataclass(frozen=True)
class DegradationDetected(AutonomyEvent):
    name: str = "degradation_detected"


@dataclass(frozen=True)
class RecoveryDetected(AutonomyEvent):
    name: str = "recovery_detected"


@dataclass(frozen=True)
class KillSwitchTriggered(AutonomyEvent):
    name: str = "killswitch_triggered"


@dataclass(frozen=True)
class LiveArmRequested(AutonomyEvent):
    """Petición externa de armado live.

    F17–F20: el orquestador la **rechaza** por política — registra
    el evento en telemetría pero no transiciona a ``LIVE_ARMED``.
    """

    name: str = "live_arm_requested"


@dataclass(frozen=True)
class ExitCompleted(AutonomyEvent):
    name: str = "exit_completed"


@dataclass(frozen=True)
class MonitoringTimeout(AutonomyEvent):
    name: str = "monitoring_timeout"


__all__ = [
    "AutonomyEvent",
    "Tick",
    "OpportunityArrived",
    "StrategyBuilt",
    "FitnessReady",
    "PaperExecuted",
    "DegradationDetected",
    "RecoveryDetected",
    "KillSwitchTriggered",
    "LiveArmRequested",
    "ExitCompleted",
    "MonitoringTimeout",
]
