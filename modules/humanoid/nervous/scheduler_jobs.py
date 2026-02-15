"""Ensure Nervous System scheduler job."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def ensure_nervous_jobs() -> None:
    """Create nervous_cycle job if enabled and not already present. Update interval if exists."""
    if not _env_bool("NERVOUS_ENABLED", True):
        return
    if not _env_bool("SCHED_ENABLED", True):
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=80) or []
        interval_sec = int(os.getenv("NERVOUS_INTERVAL_SECONDS", "10") or 10)
        interval_sec = max(5, min(interval_sec, 120))
        mode = os.getenv("NERVOUS_MODE", "auto").strip() or "auto"

        existing = next((j for j in jobs if j.get("name") == "nervous_cycle"), None)
        if existing:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, payload_json = ?, updated_ts = ? WHERE id = ?",
                (
                    interval_sec,
                    json.dumps({"mode": mode}),
                    datetime.now(timezone.utc).isoformat(),
                    existing.get("id"),
                ),
            )
            conn.commit()
            return

        now = datetime.now(timezone.utc).isoformat()
        db.create_job(
            JobSpec(
                name="nervous_cycle",
                kind="nervous_cycle",
                payload={"mode": mode},
                run_at=now,
                interval_seconds=interval_sec,
            )
        )
    except Exception:
        pass

