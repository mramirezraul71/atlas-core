"""FastAPI router for humanoid: status, plan, update-check, update-apply. Responses: {ok, data, ms, error}."""
from __future__ import annotations

import os
import time
from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from modules.humanoid import get_humanoid_kernel

router = APIRouter(prefix="/humanoid", tags=["humanoid"])


def _humanoid_enabled() -> bool:
    v = os.getenv("HUMANOID_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _resp(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None, **extra: Any) -> dict:
    out: dict = {"ok": ok, "data": data, "ms": ms, "error": error}
    out.update(extra)
    return out


@router.post("/makeplay/scan")
def humanoid_makeplay_scan() -> dict:
    """Scanner MakePlay: snapshot estado ATLAS -> webhook. Para feedback loop continuo."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.comms.makeplay_scanner import run_scan
        r = run_scan()
        ms = int((time.perf_counter() - t0) * 1000)
        snap = r.get("snapshot", {})
        return _resp(ok=r.get("ok", True), data=snap, ms=ms, webhook_pushed=snap.get("webhook_pushed", False))
    except Exception as e:
        return _resp(ok=False, error=str(e), ms=int((time.perf_counter() - t0) * 1000))


@router.get("/self-model")
def humanoid_self_model() -> dict:
    """Autoconocimiento: anatomía del sistema, cerebro, nervios, órganos, dependencias."""
    try:
        from modules.humanoid.self_model import get_manifest
        return _resp(ok=True, data=get_manifest())
    except Exception as e:
        return _resp(ok=False, error=str(e))


@router.get("/status")
def humanoid_status() -> dict:
    """Full humanoid status: modules, health, module_states, latencies."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    t0 = time.perf_counter()
    k = get_humanoid_kernel()
    raw = k.status()
    ms = int((time.perf_counter() - t0) * 1000)
    return _resp(
        ok=raw.get("ok", True),
        data={
            "modules": raw.get("modules", []),
            "health": raw.get("health", {}),
            "module_states": raw.get("module_states", raw.get("health", {}).get("modules", {})),
            "latencies_ms": raw.get("latencies_ms", {}),
        },
        ms=ms,
        error=None,
        module_states=raw.get("module_states", {}),
    )


class PlanBody(BaseModel):
    goal: str = Field(..., min_length=1)
    fast: bool = Field(default=True, description="If true, force FAST model to avoid timeout")


@router.post("/plan")
def humanoid_plan(body: PlanBody) -> dict:
    """Plan steps for a goal (LLM-backed). Returns {ok, data: {steps}, ms, error}. Timeout 15s."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    t0 = time.perf_counter()
    try:
        k = get_humanoid_kernel()
        autonomy = k.registry.get("autonomy")
        if not autonomy or not hasattr(autonomy, "planner"):
            return _resp(False, data={"steps": []}, ms=int((time.perf_counter() - t0) * 1000), error="autonomy module not available")
        result = autonomy.planner.plan(body.goal, fast=body.fast)
        if result.get("ok"):
            autonomy.tracker.set_goal(body.goal, result.get("steps"))
        ms = int((time.perf_counter() - t0) * 1000)
        if result.get("error") == "planner_timeout":
            return _resp(ok=False, data={"steps": []}, ms=ms, error="planner_timeout")
        return _resp(
            ok=result.get("ok", False),
            data={"steps": result.get("steps", [])},
            ms=ms,
            error=result.get("error"),
        )
    except (TimeoutError, Exception) as e:
        err = str(e).lower()
        ms = int((time.perf_counter() - t0) * 1000)
        if "timeout" in err or "timed out" in err:
            return _resp(ok=False, data={"steps": []}, ms=ms, error="planner_timeout")
        return _resp(ok=False, data={"steps": []}, ms=ms, error=str(e))


class UpdateCheckBody(BaseModel):
    required_packages: Optional[List[str]] = Field(default=None, description="List of pip package names to check")


@router.post("/update-check")
def humanoid_update_check(body: UpdateCheckBody) -> dict:
    """Check env and produce update plan (no installs). Returns {ok, data, ms, error, deps_missing, plan_commands}."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    t0 = time.perf_counter()
    k = get_humanoid_kernel()
    update_mod = k.registry.get("update")
    if not update_mod or not hasattr(update_mod, "updater"):
        return _resp(False, data=None, ms=int((time.perf_counter() - t0) * 1000), error="update module not available")
    required = body.required_packages or ["fastapi", "uvicorn", "httpx", "pydantic"]
    plan_result = update_mod.updater.plan(required)
    ms = int((time.perf_counter() - t0) * 1000)
    return _resp(
        ok=plan_result.get("ok", False),
        data=plan_result,
        ms=ms,
        error=plan_result.get("error"),
        deps_missing=plan_result.get("missing", []),
        plan_commands=plan_result.get("plan_commands", plan_result.get("install_plan", [])),
    )


class UpdateApplyBody(BaseModel):
    required_packages: Optional[List[str]] = Field(default=None, description="Packages to ensure installed")
    force: bool = Field(default=False, description="If true, run installs even when UPDATE_APPLY=false")


@router.post("/update-apply")
def humanoid_update_apply(body: UpdateApplyBody) -> dict:
    """Apply installs only if UPDATE_APPLY=true or force=true. Returns {ok, data, ms, error, applied_commands}."""
    if not _humanoid_enabled():
        raise HTTPException(status_code=503, detail="HUMANOID_ENABLED=false")
    k = get_humanoid_kernel()
    update_mod = k.registry.get("update")
    if not update_mod or not hasattr(update_mod, "updater"):
        return _resp(False, data=None, ms=0, error="update module not available")
    required = body.required_packages or ["fastapi", "uvicorn", "httpx", "pydantic"]
    result = update_mod.updater.apply(required, force=body.force)
    return _resp(
        ok=result.get("ok", False),
        data={"snapshot": result.get("snapshot"), "applied_commands": result.get("applied_commands", [])},
        ms=result.get("ms", 0),
        error=result.get("error"),
        applied_commands=result.get("applied_commands", []),
    )
