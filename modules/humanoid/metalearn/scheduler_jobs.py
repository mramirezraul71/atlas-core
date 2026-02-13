"""Ensure metalearn_cycle scheduler job when METALEARN_ENABLED and SCHED_ENABLED."""
from __future__ import annotations

import os
from datetime import datetime, timezone


def _metalearn_enabled() -> bool:
    return os.getenv("METALEARN_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def ensure_metalearn_jobs() -> None:
    """Create metalearn_cycle job if enabled and not already present."""
    if not _metalearn_enabled() or not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec
        db = get_scheduler_db()
        jobs = db.list_jobs()
        names = {j.get("name") for j in (jobs or [])}
        if "metalearn_cycle" in names:
            return
        interval_min = int(os.getenv("METALEARN_UPDATE_INTERVAL_MINUTES", "60") or 60)
        interval_sec = max(60, interval_min * 60)
        now = datetime.now(timezone.utc).isoformat()
        db.create_job(JobSpec(
            name="metalearn_cycle",
            kind="metalearn_cycle",
            payload={},
            run_at=now,
            interval_seconds=interval_sec,
        ))
    except Exception:
        pass
