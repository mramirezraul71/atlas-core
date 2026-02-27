"""Meta-learning HTTP API."""
from __future__ import annotations

from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/metalearn", tags=["metalearn"])


def _enabled() -> bool:
    import os
    return os.getenv("METALEARN_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _resp(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None) -> dict:
    return {"ok": ok, "data": data, "ms": ms, "error": error}


@router.get("/status")
def metalearn_status() -> dict:
    """enabled, last_update_ts, sample_count, rule_count, last_changes_summary."""
    if not _enabled():
        return _resp(ok=True, data={"enabled": False, "error": "METALEARN_ENABLED=false"}, ms=0)
    try:
        from modules.humanoid.metalearn.cycle import get_status
        st = get_status()
        return _resp(ok=True, data=st, ms=0)
    except Exception as e:
        return _resp(ok=False, data={}, ms=0, error=str(e))


@router.post("/run")
def metalearn_run() -> dict:
    """Force stats update + tuning (if allowed). Non-blocking; returns quickly."""
    import time
    t0 = time.perf_counter()
    if not _enabled():
        raise HTTPException(status_code=503, detail="METALEARN_ENABLED=false")
    try:
        from modules.humanoid.metalearn.cycle import run_cycle
        result = run_cycle()
        ms = int((time.perf_counter() - t0) * 1000)
        return _resp(ok=result.get("ok", False), data=result, ms=ms, error=result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _resp(ok=False, data={}, ms=ms, error=str(e))


@router.get("/rules")
def metalearn_rules() -> dict:
    """Top 20 active rules with evidence (approve_rate, success_rate)."""
    if not _enabled():
        return _resp(ok=True, data={"rules": []}, ms=0)
    try:
        from modules.humanoid.metalearn.db import get_rules
        rules = get_rules(limit=20)
        return _resp(ok=True, data={"rules": rules}, ms=0)
    except Exception as e:
        return _resp(ok=False, data={"rules": []}, ms=0, error=str(e))


@router.get("/report/latest")
def metalearn_report_latest() -> dict:
    """Path and summary of latest meta-learning report."""
    if not _enabled():
        return _resp(ok=True, data={"path": None, "summary": "Meta-learning disabled"}, ms=0)
    try:
        from modules.humanoid.metalearn.reporter import get_latest_report_path
        path = get_latest_report_path()
        if not path:
            return _resp(ok=True, data={"path": None, "summary": "No reports yet"}, ms=0)
        from pathlib import Path
        p = Path(path)
        content = p.read_text(encoding="utf-8", errors="ignore")[:2000] if p.exists() else ""
        summary = "\n".join(content.split("\n")[:12]) if content else ""
        return _resp(ok=True, data={"path": path, "summary": summary}, ms=0)
    except Exception as e:
        return _resp(ok=False, data={}, ms=0, error=str(e))


class RollbackBody(BaseModel):
    snapshot_id: str = Field(..., min_length=1)


@router.post("/rollback")
def metalearn_rollback(body: RollbackBody) -> dict:
    """Rollback tuning to a previous snapshot."""
    if not _enabled():
        raise HTTPException(status_code=503, detail="METALEARN_ENABLED=false")
    try:
        from modules.humanoid.metalearn.rollback import rollback
        result = rollback(body.snapshot_id)
        return _resp(ok=result.get("ok", False), data=result, ms=0, error=result.get("error"))
    except Exception as e:
        return _resp(ok=False, data={}, ms=0, error=str(e))
