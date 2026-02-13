"""Ensure GA scheduler jobs exist when SCHED_ENABLED and GOVERNED_AUTONOMY_ENABLED."""
from __future__ import annotations

import os
from datetime import datetime, timezone


def _ga_enabled() -> bool:
    return os.getenv("GOVERNED_AUTONOMY_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def ensure_ga_jobs() -> None:
    """Create ga_cycle job if enabled and not already present."""
    if not _ga_enabled() or not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec
        db = get_scheduler_db()
        jobs = db.list_jobs()
        names = {j.get("name") for j in (jobs or [])}
        if "ga_cycle" in names:
            return
        interval_min = int(os.getenv("GA_CYCLE_INTERVAL_MINUTES", "60") or 60)
        interval_sec = max(60, interval_min * 60)
        max_findings = int(os.getenv("GA_MAX_FINDINGS", "10") or 10)
        mode = os.getenv("GA_MODE", "plan_only").strip()
        now = datetime.now(timezone.utc).isoformat()
        db.create_job(JobSpec(
            name="ga_cycle",
            kind="ga_cycle",
            payload={"scope": "all", "mode": mode, "max_findings": max_findings},
            run_at=now,
            interval_seconds=interval_sec,
        ))
    except Exception:
        pass
