"""FastAPI router for Nervous System."""

from __future__ import annotations

import time
from fastapi import APIRouter
from pydantic import BaseModel, Field

from .engine import get_nervous_status, run_nervous_cycle

router = APIRouter(prefix="/nervous", tags=["nervous"])


class NervousRunBody(BaseModel):
    mode: str = Field(default="auto")


@router.get("/status")
def nervous_status(limit: int = 50) -> dict:
    return {"ok": True, "data": get_nervous_status(limit=limit)}


@router.post("/cycle/run")
def nervous_cycle_run(body: NervousRunBody | None = None) -> dict:
    body = body or NervousRunBody()
    out = run_nervous_cycle(mode=body.mode)
    return {"ok": out.get("ok", False), "data": out}


@router.get("/brain/audit")
def nervous_brain_audit() -> dict:
    """
    Auditoría técnica del cerebro: capacidad (modelos), velocidad (latencias p95),
    nivel de reacción (colas/errores), coherencia (brain validators) y conectividad (cuerpo/nerve).
    """
    t0 = time.perf_counter()
    data: dict = {"ok": True}
    try:
        from modules.humanoid.ai.brain_state import get_brain_state
        data["brain_state"] = get_brain_state()
    except Exception:
        data["brain_state"] = {}
    try:
        from modules.humanoid.ai.status import get_ai_status
        data["ai_status"] = get_ai_status()
    except Exception:
        data["ai_status"] = {}
    try:
        from modules.humanoid import get_humanoid_kernel
        data["humanoid"] = (get_humanoid_kernel().status() or {}).get("health", {})
    except Exception:
        data["humanoid"] = {}
    try:
        from modules.humanoid.metrics import get_metrics_store
        m = get_metrics_store().snapshot()
        lat = (m.get("latencies") or {}) if isinstance(m, dict) else {}
        # extract critical endpoints p95
        crit = {}
        for k in ("request:/status", "request:/health", "request:/api/brain/state", "request:/api/robot/status", "request:/update/check"):
            if k in lat:
                crit[k] = lat.get(k)
        data["metrics"] = {"critical_latencies": crit, "counters": (m.get("counters") or {}) if isinstance(m, dict) else {}}
    except Exception:
        data["metrics"] = {}
    try:
        from modules.humanoid.nerve import nerve_eyes_status
        data["nerve_eyes"] = nerve_eyes_status()
    except Exception:
        data["nerve_eyes"] = {}
    ms = int((time.perf_counter() - t0) * 1000)
    return {"ok": True, "data": data, "ms": ms, "error": None}

