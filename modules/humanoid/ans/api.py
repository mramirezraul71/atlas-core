"""ANS API: status, incidents, report, run-now."""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional

router = APIRouter(prefix="/ans", tags=["ANS"])


@router.get("/status")
def ans_status():
    from .engine import get_ans_status
    return get_ans_status()


@router.get("/incidents")
def ans_incidents(status: Optional[str] = None, limit: int = 50):
    try:
        from .incident import get_incidents
        items = get_incidents(status=status, limit=limit)
        return {"ok": True, "data": items}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/report/latest")
def ans_report_latest():
    try:
        from .reporter import get_latest_report
        path = get_latest_report()
        return {"ok": True, "path": path}
    except Exception as e:
        return {"ok": False, "path": "", "error": str(e)}


class RunNowBody(BaseModel):
    mode: Optional[str] = None


@router.post("/run-now")
def ans_run_now(body: Optional[RunNowBody] = None):
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "ans", "ans_run_now", None)
        if not decision.allow:
            return {"ok": False, "error": decision.reason}
        from .engine import run_ans_cycle
        mode = body.mode if body else None
        r = run_ans_cycle(mode=mode, timeout_sec=60)
        return r
    except Exception as e:
        return {"ok": False, "error": str(e)}
