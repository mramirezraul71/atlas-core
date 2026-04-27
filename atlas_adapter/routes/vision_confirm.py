"""Endpoint /vision/confirm — F7.

Expone la decisión de ``VisionTimingGate`` como REST GET.

Diseño paper-first:
- Si ``ATLAS_VISION_GATE_ENABLED`` no es true, el router se monta pero
  responde con ``decision="allow"`` y ``degraded=true`` para no bloquear
  pruebas en paper.
- Si está habilitado, instancia ``VisionTimingGate`` con cámara
  ``CAMERA_UNAVAILABLE`` por defecto (la integración Insta360 viene en F7.b).
"""
from __future__ import annotations

import os
from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter, Query

from atlas_code_quant.vision.timing_gate import (
    GateInput,
    VisionTimingGate,
)


def _env_bool(name: str, default: bool) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in {"1", "true", "yes", "on"}


def build_vision_confirm_router() -> APIRouter:
    router = APIRouter(tags=["Vision"])
    gate = VisionTimingGate()

    @router.get("/vision/confirm")
    def vision_confirm(
        symbol: str = Query(..., min_length=1, max_length=16),
        intent: Literal["entry", "exit"] = Query("entry"),
        strategy: str = Query("unknown", max_length=64),
        requires_visual_confirmation: bool = Query(False),
    ) -> dict:
        if not _env_bool("ATLAS_VISION_GATE_ENABLED", False):
            return {
                "decision": "allow",
                "confidence": 0.50,
                "imbalance_side": "neutral",
                "pattern": "unknown",
                "ts": datetime.now(timezone.utc).isoformat(),
                "degraded": True,
                "camera_state": "CAMERA_UNAVAILABLE",
                "reason": "vision_gate_disabled_paper_default",
                "symbol": symbol.upper(),
                "intent": intent,
                "strategy": strategy,
            }

        result = gate.evaluate(
            GateInput(
                symbol=symbol.upper(),
                intent=intent,
                strategy=strategy,
                requires_visual_confirmation=requires_visual_confirmation,
                camera_state="CAMERA_UNAVAILABLE",
                imbalance=None,
                pattern="unknown",
            )
        )
        return {
            "decision": result.decision,
            "confidence": result.confidence,
            "imbalance_side": result.imbalance_side,
            "pattern": result.pattern,
            "ts": result.timestamp,
            "degraded": result.degraded,
            "camera_state": result.camera_state,
            "reason": result.reason,
            "symbol": symbol.upper(),
            "intent": intent,
            "strategy": strategy,
        }

    return router
