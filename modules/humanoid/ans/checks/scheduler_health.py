"""Scheduler health check."""
from __future__ import annotations

import os


def run() -> dict:
    if os.getenv("SCHED_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
        return {"ok": True, "check_id": "scheduler_health", "message": "scheduler disabled", "details": {}, "severity": "low"}
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        jobs = db.list_jobs(limit=5)
        return {"ok": True, "check_id": "scheduler_health", "message": "ok", "details": {"jobs_count": len(jobs)}, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "scheduler_health", "message": str(e), "details": {"error": str(e)}, "severity": "med", "suggested_heals": ["restart_scheduler"]}
