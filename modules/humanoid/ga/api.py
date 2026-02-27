"""GA HTTP API: /ga/run, /ga/status, /ga/report/latest."""
from __future__ import annotations

import os
from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/ga", tags=["ga"])


def _ga_enabled() -> bool:
    return os.getenv("GOVERNED_AUTONOMY_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _resp(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None, **extra: Any) -> dict:
    out: dict = {"ok": ok, "data": data, "ms": ms, "error": error}
    out.update(extra)
    return out


class GaRunBody(BaseModel):
    scope: str = Field(default="all", description="runtime|repo|all")
    mode: str = Field(default="plan_only", description="plan_only|controlled|auto")
    max_findings: Optional[int] = Field(default=None, description="Max findings to process")


@router.post("/run")
def ga_run(body: GaRunBody) -> dict:
    """
    Run GA cycle. scope: runtime|repo|all. mode: plan_only|controlled|auto.
    Returns {ok, data: {findings, plan, executed, approvals_created, report_path, correlation_id}, ms, error}.
    Timeout-friendly: completes in <15s typically.
    """
    if not _ga_enabled():
        raise HTTPException(status_code=503, detail="GOVERNED_AUTONOMY_ENABLED=false")
    import time
    t0 = time.perf_counter()
    try:
        from modules.humanoid.ga.cycle import run_cycle
        result = run_cycle(
            scope=body.scope or "all",
            mode=body.mode or "plan_only",
            max_findings=body.max_findings,
        )
        ms = int((time.perf_counter() - t0) * 1000)
        if not result.get("ok"):
            return _resp(ok=False, data=result.get("data", {}), ms=ms, error=result.get("error"))
        return _resp(ok=True, data=result.get("data", {}), ms=ms, error=None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _resp(ok=False, data={}, ms=ms, error=str(e))


@router.get("/status")
def ga_status() -> dict:
    """Last run timestamp, last report path, approvals pending count, mode."""
    if not _ga_enabled():
        raise HTTPException(status_code=503, detail="GOVERNED_AUTONOMY_ENABLED=false")
    try:
        from modules.humanoid.ga.cycle import get_status
        st = get_status()
        return _resp(ok=True, data=st, ms=0, error=None)
    except Exception as e:
        return _resp(ok=False, data={}, ms=0, error=str(e))


@router.get("/report/latest")
def ga_report_latest() -> dict:
    """Path and summary of latest GA report."""
    if not _ga_enabled():
        raise HTTPException(status_code=503, detail="GOVERNED_AUTONOMY_ENABLED=false")
    try:
        from modules.humanoid.ga.reporter import get_latest_report_path
        path = get_latest_report_path()
        if not path:
            return _resp(ok=True, data={"path": None, "summary": "No reports yet"}, ms=0, error=None)
        from pathlib import Path
        p = Path(path)
        content = p.read_text(encoding="utf-8", errors="ignore")[:2000] if p.exists() else ""
        summary = content.split("\n")[0:8] if content else []
        return _resp(ok=True, data={"path": path, "summary": "\n".join(summary)}, ms=0, error=None)
    except Exception as e:
        return _resp(ok=False, data={}, ms=0, error=str(e))
