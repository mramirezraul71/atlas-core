"""Vision Timing Gate scaffold (F1).

No conecta cámara ni endpoints nuevos en esta fase.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Literal


GateDecision = Literal["allow", "delay", "block", "force_exit"]


@dataclass(slots=True)
class GateInput:
    symbol: str
    intent: str = "entry"
    strategy: str = "unknown"
    requires_visual_confirmation: bool = False


@dataclass(slots=True)
class GateOutput:
    decision: GateDecision
    confidence: float
    reason: str
    degraded: bool
    timestamp: str


class VisionTimingGate:
    """Stub seguro: degrada explícitamente cuando no hay motor visual real."""

    def evaluate(self, gate_input: GateInput) -> GateOutput:
        if gate_input.requires_visual_confirmation:
            return GateOutput(
                decision="block",
                confidence=0.0,
                reason="vision_gate_stub_requires_visual_confirmation",
                degraded=True,
                timestamp=datetime.now(timezone.utc).isoformat(),
            )
        return GateOutput(
            decision="allow",
            confidence=0.55,
            reason="vision_gate_stub_degraded_allow",
            degraded=True,
            timestamp=datetime.now(timezone.utc).isoformat(),
        )
