"""Atlas Code Quant — VisionTimingGate (F18).

Gate institucional de timing visual para decisiones de trading.
Devuelve un veredicto canónico ``allow | delay | block | force_exit``
con marca opcional de degradación.

Política F18 (verbatim del usuario):

    * Si cámara unavailable y requires_visual_confirmation=False
      → decision=allow, degraded=True.
    * Si cámara unavailable y requires_visual_confirmation=True
      → decision=block, degraded=True.

Se mantiene independiente del orquestador F17 — éste lo consume a
través del `VisionGate` de ``autonomy.gates.orchestrator_gates``.

Reglas duras F18:

* NO live: el gate puede usarse en F18–F20 sólo dentro del pipeline
  paper. ``allow=True`` aquí no autoriza live trading.
* NO toca cámaras reales si ``capture_provider`` es None — devuelve
  ``camera_status="unavailable"`` y aplica la política F18.
* Defensivo: ``evaluate`` nunca lanza.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping

logger = logging.getLogger("atlas.code_quant.vision.timing_gate")


__all__ = [
    "VisionDecision",
    "GateInput",
    "GateOutput",
    "VisionTimingGate",
]


class VisionDecision(str, Enum):
    ALLOW = "allow"
    DELAY = "delay"
    BLOCK = "block"
    FORCE_EXIT = "force_exit"


@dataclass(frozen=True)
class GateInput:
    """Entrada estándar del VisionTimingGate (F18).

    Attributes:
        opportunity: payload Radar (dict simplificado, no acopla
            tipos canónicos para mantener este módulo aislado).
        strategy_id: id del intent/estructura evaluada.
        requires_visual_confirmation: si la estrategia exige
            confirmación visual (por política o por horizonte corto).
        market_open: hint contextual; si False, el gate aplica una
            política conservadora.
        extras: payload extensible.
    """

    opportunity: Mapping[str, Any] = field(default_factory=dict)
    strategy_id: str = ""
    requires_visual_confirmation: bool = False
    market_open: bool = True
    extras: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class GateOutput:
    """Salida estándar del VisionTimingGate (F18)."""

    decision: VisionDecision
    reason: str
    degraded: bool = False
    camera_status: str = "unknown"  # "ok" | "unavailable" | "error"
    evidence: Mapping[str, Any] = field(default_factory=dict)


CaptureProbe = Callable[[], Mapping[str, Any]]
"""Función inyectable que devuelve un dict con al menos:
    {"ok": bool, "source": str, "error": Optional[str]}.
"""

PatternEvaluator = Callable[[GateInput, Mapping[str, Any]], GateOutput]
"""Evaluador inyectable que recibe el input y el resultado de la captura
y devuelve un GateOutput. Permite testear políticas más complejas (delay,
force_exit) sin requerir hardware real ni modelos visuales en F18.
"""


def _default_capture_probe() -> Mapping[str, Any]:
    """Probe defensivo: nunca toca hardware real.

    Devuelve ``ok=False`` con ``source="none"``; quien quiera
    integrar Insta360/Webcam debe inyectar su propio probe.
    """
    return {"ok": False, "source": "none", "error": "no_capture_probe_injected"}


def _log_vision_decision(inp: "GateInput", out: "GateOutput") -> None:
    evidence = dict(out.evidence or {})
    payload = {
        "type": "vision_decision",
        "decision": out.decision.value,
        "degraded": bool(out.degraded),
        "confidence": evidence.get("confidence"),
        "pattern": evidence.get("pattern"),
        "imbalanceside": evidence.get("imbalanceside", "neutral"),
        "requires_visual_confirmation": bool(inp.requires_visual_confirmation),
        "camera_state": out.camera_status or "unknown",
        "trace_id": evidence.get("trace_id"),
        "symbol": evidence.get("symbol") or inp.opportunity.get("symbol"),
        "strategy_id": evidence.get("strategy_id") or inp.strategy_id or None,
        "ts": evidence.get("ts") or datetime.now(timezone.utc).isoformat(),
    }
    logger.info("vision_decision %s", payload)


def _default_pattern_evaluator(
    inp: GateInput, capture: Mapping[str, Any]
) -> GateOutput:
    """Evaluador por defecto: si la captura está OK, allow. Si no,
    aplica la política F18 (allow/block según requires_visual_confirmation)."""
    cam_ok = bool(capture.get("ok"))
    if cam_ok:
        return GateOutput(
            decision=VisionDecision.ALLOW,
            reason="camera_ok_no_pattern_engine",
            degraded=False,
            camera_status="ok",
            evidence={"source": str(capture.get("source", ""))},
        )
    if inp.requires_visual_confirmation:
        return GateOutput(
            decision=VisionDecision.BLOCK,
            reason="camera_unavailable_visual_required",
            degraded=True,
            camera_status="unavailable",
            evidence={"error": str(capture.get("error", ""))},
        )
    return GateOutput(
        decision=VisionDecision.ALLOW,
        reason="camera_unavailable_visual_optional",
        degraded=True,
        camera_status="unavailable",
        evidence={"error": str(capture.get("error", ""))},
    )


class VisionTimingGate:
    """Gate institucional de timing visual (F18)."""

    def __init__(
        self,
        *,
        capture_probe: CaptureProbe | None = None,
        pattern_evaluator: PatternEvaluator | None = None,
    ) -> None:
        self._probe: CaptureProbe = capture_probe or _default_capture_probe
        self._eval: PatternEvaluator = (
            pattern_evaluator or _default_pattern_evaluator
        )

    def evaluate(self, inp: GateInput | None) -> GateOutput:
        """Evalúa el gate. Defensivo: nunca lanza."""
        if inp is None:
            out = GateOutput(
                decision=VisionDecision.BLOCK,
                reason="gate_input_missing",
                degraded=True,
                camera_status="unknown",
            )
            logger.info(
                "vision_decision %s",
                {
                    "type": "vision_decision",
                    "decision": out.decision.value,
                    "degraded": bool(out.degraded),
                    "confidence": None,
                    "pattern": None,
                    "imbalanceside": "neutral",
                    "requires_visual_confirmation": None,
                    "camera_state": out.camera_status or "unknown",
                    "trace_id": None,
                    "symbol": None,
                    "strategy_id": None,
                    "ts": datetime.now(timezone.utc).isoformat(),
                },
            )
            return out
        try:
            cap = self._probe() or {}
        except Exception as exc:  # noqa: BLE001
            logger.warning("vision_timing_gate: capture_probe raised: %s", exc)
            cap = {"ok": False, "source": "error", "error": str(exc)}
        try:
            out = self._eval(inp, cap)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "vision_timing_gate: pattern_evaluator raised: %s", exc
            )
            out = GateOutput(
                decision=VisionDecision.BLOCK,
                reason=f"pattern_evaluator_raised:{exc}",
                degraded=True,
                camera_status=str(cap.get("source", "unknown")),
            )
            _log_vision_decision(inp, out)
            return out
        if not isinstance(out, GateOutput):
            fallback = GateOutput(
                decision=VisionDecision.BLOCK,
                reason="pattern_evaluator_invalid_return",
                degraded=True,
                camera_status=str(cap.get("source", "unknown")),
            )
            _log_vision_decision(inp, fallback)
            return fallback
        _log_vision_decision(inp, out)
        return out
