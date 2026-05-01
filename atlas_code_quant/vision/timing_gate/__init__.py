"""vision.timing_gate — gate institucional de timing visual (F18).

F1 scaffold consolidado en F18 con la implementación real:
``VisionTimingGate``, ``GateInput``, ``GateOutput``, ``VisionDecision``.
Uso permitido sólo dentro del pipeline paper (F17 orchestrator).
"""

from atlas_code_quant.vision.timing_gate.gate import (
    GateInput,
    GateOutput,
    VisionDecision,
    VisionTimingGate,
)

__all__ = [
    "GateInput",
    "GateOutput",
    "VisionDecision",
    "VisionTimingGate",
]
