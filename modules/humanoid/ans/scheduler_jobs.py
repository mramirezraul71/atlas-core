"""Ensure ANS scheduler job when ANS_ENABLED and SCHED_ENABLED."""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone


def _ans_enabled() -> bool:
    return os.getenv("ANS_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def ensure_ans_jobs() -> None:
    """Create ans_cycle job if enabled and not already present. Update interval if exists."""
    if not _ans_enabled() or not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec
        db = get_scheduler_db()
        jobs = db.list_jobs(limit=50) or []
        interval_sec = int(os.getenv("ANS_INTERVAL_SECONDS", "15") or 15)
        interval_sec = max(10, min(interval_sec, 300))
        mode = os.getenv("ANS_MODE", "auto").strip()
        ans_job = next((j for j in jobs if j.get("name") == "ans_cycle"), None)
        if ans_job:
            # Actualizar intervalo si cambió (reinicio aplica nueva velocidad)
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, payload_json = ?, updated_ts = ? WHERE id = ?",
                (interval_sec, json.dumps({"mode": mode, "timeout_sec": 60}),
                 datetime.now(timezone.utc).isoformat(), ans_job.get("id")),
            )
            conn.commit()
            return
        now = datetime.now(timezone.utc).isoformat()
        db.create_job(JobSpec(
            name="ans_cycle",
            kind="ans_cycle",
            payload={"mode": mode, "timeout_sec": 60},
            run_at=now,
            interval_seconds=interval_sec,
        ))
    except Exception:
        pass
