"""Trading bridge routes that keep camera integration outside atlas_code_quant."""
from __future__ import annotations

from typing import Any, Optional

from atlas_adapter.services.trading_quant_bridge import (
    get_quant_bridge_status,
    run_quant_vision_cycle,
)
from fastapi import APIRouter
from pydantic import BaseModel, Field, field_validator

router = APIRouter(tags=["Trading"])


class TradingQuantVisionCycleBody(BaseModel):
    order: dict[str, Any] = Field(default_factory=dict)
    action: str = "evaluate"
    include_vision: bool = True
    capture_target: str = "nexus:camera"
    screen_target: Optional[dict[str, Any]] = None
    calib_path: Optional[str] = None
    source: str = "camera"
    quant_capture_context: bool = False
    timeout_sec: int = Field(default=20, ge=1, le=120)

    @field_validator("action")
    @classmethod
    def validate_action(cls, value: str) -> str:
        normalized = str(value or "evaluate").strip().lower()
        if normalized not in {"evaluate", "preview", "submit"}:
            raise ValueError("action must be evaluate, preview or submit")
        return normalized


def build_router() -> APIRouter:
    local_router = APIRouter()
    local_router.include_router(router)

    @local_router.get("/api/trading/quant/vision-bridge/status")
    def quant_vision_bridge_status():
        """Return bridge readiness and handoff prompt location."""
        return get_quant_bridge_status()

    @local_router.post("/api/trading/quant/vision-cycle")
    def trading_quant_vision_cycle(body: TradingQuantVisionCycleBody):
        """Run a quant operation cycle plus optional camera alignment and capture."""
        return run_quant_vision_cycle(
            order=body.order if isinstance(body.order, dict) else {},
            action=body.action,
            include_vision=body.include_vision,
            capture_target=body.capture_target,
            screen_target=body.screen_target,
            calib_path=body.calib_path,
            source=body.source,
            quant_capture_context=body.quant_capture_context,
            timeout_sec=body.timeout_sec,
        )

    return local_router
