"""autonomy.gates.contracts — Stubs F1 (scaffold, NO usar en runtime).

Define las dataclasses canónicas que usarán los gates institucionales
(risk_gate, vision_gate, market_hours_gate, etc.) en fases posteriores.

Estado: **scaffold F1** — TODO implementación real en F6 (vision_gate)
y F7 (FSM transitions). NINGÚN módulo productivo importa este archivo en F1.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


class GateDecision(str, Enum):
    """Decisión canónica de cualquier gate institucional.

    TODO F6+: reemplazar el actual boolean ``allow_entry`` en
    ``atlas_code_quant/options/options_intent_router.py`` por este enum,
    manteniendo wrapper de compatibilidad.
    """

    ALLOW = "allow"
    DELAY = "delay"
    BLOCK = "block"
    FORCE_EXIT = "force_exit"


@dataclass(frozen=True)
class GateInput:
    """Entrada estándar de un gate.

    Attributes:
        trace_id: ID de traza end-to-end (correlación con telemetry/journal).
        symbol: símbolo evaluado.
        intent: intención de trading (str libre en F1, enum en F6+).
        context: payload arbitrario con datos extra (regime, vision, risk).
    """

    trace_id: str
    symbol: str
    intent: str
    context: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class GateOutput:
    """Salida estándar de un gate.

    Attributes:
        decision: ALLOW / DELAY / BLOCK / FORCE_EXIT.
        reason: razón legible.
        retry_after_seconds: si DELAY, sugerencia de espera. None en otros casos.
        evidence: evidencia opcional (capturas, métricas, refs a journal).
    """

    decision: GateDecision
    reason: str
    retry_after_seconds: float | None = None
    evidence: Mapping[str, Any] = field(default_factory=dict)


__all__ = [
    "GateDecision",
    "GateInput",
    "GateOutput",
]
