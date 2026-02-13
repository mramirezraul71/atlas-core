"""Scheduler jobs para MakePlay: scanner permanente."""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone


def _enabled() -> bool:
    url = (os.getenv("MAKEPLAY_WEBHOOK_URL") or os.getenv("EXTERNAL_WEBHOOK_URL") or "").strip()
    return bool(url.startswith("http")) or os.getenv("MAKEPLAY_ENABLED", "").strip().lower() in ("1", "true", "yes")


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def ensure_makeplay_jobs() -> None:
    """Crea/actualiza job scanner permanente si MakePlay habilitado."""
    if not _enabled() or not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec
        db = get_scheduler_db()
        jobs = db.list_jobs(limit=50) or []
        interval_sec = int(os.getenv("MAKEPLAY_SCAN_INTERVAL_SECONDS", "60") or 60)
        interval_sec = max(30, min(interval_sec, 3600))  # 30s a 1h
        scanner_job = next((j for j in jobs if j.get("name") == "makeplay_scanner"), None)
        if scanner_job:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, updated_ts = ? WHERE id = ?",
                (interval_sec, datetime.now(timezone.utc).isoformat(), scanner_job.get("id")),
            )
            conn.commit()
            return
        now = datetime.now(timezone.utc).isoformat()
        db.create_job(JobSpec(
            name="makeplay_scanner",
            kind="makeplay_scanner",
            payload={},
            run_at=now,
            interval_seconds=interval_sec,
        ))
    except Exception:
        pass
