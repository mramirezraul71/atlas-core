"""FastAPI router for humanoid: /humanoid/status, /humanoid/plan, /humanoid/update-check."""
from __future__ import annotations

import os
from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from modules.humanoid import get_humanoid_kernel

router = APIRouter(prefix="/humanoid", tags=["humanoid"])


def _humanoid_enabled() -> bool:
    v = os.getenv("HUMANOID_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


@router.get("/status")
def humanoid_status() -> dict:
    """Full humanoid status: registered modules + health."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    k = get_humanoid_kernel()
    return k.status()


class PlanBody(BaseModel):
    goal: str = Field(..., min_length=1)


@router.post("/plan")
def humanoid_plan(body: PlanBody) -> dict:
    """Plan steps for a goal (LLM-backed). Returns {ok, steps, error}."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    k = get_humanoid_kernel()
    autonomy = k.registry.get("autonomy")
    if not autonomy or not hasattr(autonomy, "planner"):
        return {"ok": False, "steps": [], "error": "autonomy module not available"}
    result = autonomy.planner.plan(body.goal)
    if result.get("ok"):
        autonomy.tracker.set_goal(body.goal, result.get("steps"))
    return result


class UpdateCheckBody(BaseModel):
    required_packages: Optional[List[str]] = Field(default=None, description="List of pip package names to check")


@router.post("/update-check")
def humanoid_update_check(body: UpdateCheckBody) -> dict:
    """Check env (python, pip list) and produce update plan. Does NOT run installs."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    k = get_humanoid_kernel()
    update_mod = k.registry.get("update")
    if not update_mod or not hasattr(update_mod, "updater"):
        return {"ok": False, "error": "update module not available"}
    required = body.required_packages or ["fastapi", "uvicorn", "httpx", "pydantic"]
    return update_mod.updater.plan(required)
