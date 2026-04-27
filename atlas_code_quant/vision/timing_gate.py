"""Vision Timing Gate — F7.

Decide si una orden puede ejecutarse en función de:
- Estado de la cámara Insta360 (``CAMERA_OK`` / ``CAMERA_UNAVAILABLE``).
- Imbalance lateral detectado (buy/sell/neutral).
- Patrón visual de timing (``setup`` / ``confirmation`` / ``noise`` / ``unknown``).
- Flag ``requires_visual_confirmation`` del intent.

Política paper-first:
- Si la cámara está disponible y el patrón es ``confirmation``, devuelve
  ``allow`` con confianza alta.
- Si la cámara no está disponible y la orden NO requiere confirmación visual,
  devuelve ``allow`` con ``degraded:true`` y confianza moderada.
- Si la orden requiere confirmación visual y la cámara está caída, devuelve
  ``block``.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Literal

from atlas_code_quant.vision.imbalance import (
    ImbalanceSnapshot,
    classify_imbalance,
)


GateDecision = Literal["allow", "delay", "block", "force_exit"]
CameraState = Literal["CAMERA_OK", "CAMERA_UNAVAILABLE", "CAMERA_DEGRADED"]
VisualPattern = Literal["setup", "confirmation", "noise", "unknown"]


@dataclass(slots=True)
class GateInput:
    symbol: str
    intent: Literal["entry", "exit"] = "entry"
    strategy: str = "unknown"
    requires_visual_confirmation: bool = False
    camera_state: CameraState = "CAMERA_UNAVAILABLE"
    imbalance: ImbalanceSnapshot | None = None
    pattern: VisualPattern = "unknown"


@dataclass(slots=True)
class GateOutput:
    decision: GateDecision
    confidence: float
    reason: str
    degraded: bool
    timestamp: str
    imbalance_side: str = "neutral"
    pattern: VisualPattern = "unknown"
    camera_state: CameraState = "CAMERA_UNAVAILABLE"


class VisionTimingGate:
    """Política Vision sin dependencias externas (testeable)."""

    def evaluate(self, gate_input: GateInput) -> GateOutput:
        ts = datetime.now(timezone.utc).isoformat()
        side = (
            classify_imbalance(gate_input.imbalance.score)
            if gate_input.imbalance
            else "neutral"
        )

        # Salidas (exit) son siempre permitidas: prevalece protección sobre timing
        if gate_input.intent == "exit":
            return GateOutput(
                decision="allow",
                confidence=0.85,
                reason="exit_intent_bypass",
                degraded=gate_input.camera_state != "CAMERA_OK",
                timestamp=ts,
                imbalance_side=side,
                pattern=gate_input.pattern,
                camera_state=gate_input.camera_state,
            )

        # Cámara caída + requiere visual (sólo entries) → block
        if (
            gate_input.camera_state != "CAMERA_OK"
            and gate_input.requires_visual_confirmation
        ):
            return GateOutput(
                decision="block",
                confidence=0.0,
                reason="camera_unavailable_visual_required",
                degraded=True,
                timestamp=ts,
                imbalance_side=side,
                pattern=gate_input.pattern,
                camera_state=gate_input.camera_state,
            )

        # Camera OK → confianza según patrón
        if gate_input.camera_state == "CAMERA_OK":
            if gate_input.pattern == "confirmation":
                return GateOutput(
                    decision="allow",
                    confidence=0.85,
                    reason="confirmation_pattern",
                    degraded=False,
                    timestamp=ts,
                    imbalance_side=side,
                    pattern=gate_input.pattern,
                    camera_state=gate_input.camera_state,
                )
            if gate_input.pattern == "noise":
                return GateOutput(
                    decision="delay",
                    confidence=0.30,
                    reason="noise_pattern_wait_for_setup",
                    degraded=False,
                    timestamp=ts,
                    imbalance_side=side,
                    pattern=gate_input.pattern,
                    camera_state=gate_input.camera_state,
                )
            # setup / unknown
            return GateOutput(
                decision="allow",
                confidence=0.65,
                reason="setup_pattern_or_unknown",
                degraded=False,
                timestamp=ts,
                imbalance_side=side,
                pattern=gate_input.pattern,
                camera_state=gate_input.camera_state,
            )

        # Camera caída + NO requiere visual → allow degraded
        return GateOutput(
            decision="allow",
            confidence=0.55,
            reason="camera_unavailable_no_visual_required",
            degraded=True,
            timestamp=ts,
            imbalance_side=side,
            pattern=gate_input.pattern,
            camera_state=gate_input.camera_state,
        )
