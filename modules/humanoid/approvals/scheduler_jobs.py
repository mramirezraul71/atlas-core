"""Ensure approvals digest to Telegram when owner is away."""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from typing import Any, Dict, List


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")


def _telegram_enabled() -> bool:
    # Default ON if token exists; can be disabled explicitly.
    v = (os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "true") or "").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def ensure_approvals_jobs() -> None:
    """Create approvals digest job if enabled and not already present. Update interval if exists."""
    if not _sched_enabled() or not _telegram_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=80) or []
        interval_sec = int(os.getenv("TELEGRAM_APPROVALS_DIGEST_SECONDS", "180") or 180)
        interval_sec = max(60, min(interval_sec, 1800))
        reaper_interval_sec = int(os.getenv("APPROVALS_REAPER_SECONDS", "60") or 60)
        reaper_interval_sec = max(30, min(reaper_interval_sec, 900))
        job = next((j for j in jobs if j.get("name") == "approvals_digest_telegram"), None)
        reaper_job = next((j for j in jobs if j.get("name") == "approvals_expiry_reaper"), None)
        now = datetime.now(timezone.utc).isoformat()
        payload = {"limit": 8}
        reaper_payload = {"limit": 2000}
        if job:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, payload_json = ?, updated_ts = ? WHERE id = ?",
                (interval_sec, json.dumps(payload), now, job.get("id")),
            )
        else:
            db.create_job(
                JobSpec(
                    name="approvals_digest_telegram",
                    kind="approvals_digest",
                    payload=payload,
                    run_at=now,
                    interval_seconds=interval_sec,
                )
            )

        if reaper_job:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, payload_json = ?, updated_ts = ? WHERE id = ?",
                (reaper_interval_sec, json.dumps(reaper_payload), now, reaper_job.get("id")),
            )
            conn.commit()
            return

        db.create_job(
            JobSpec(
                name="approvals_expiry_reaper",
                kind="approvals_expiry_reaper",
                payload=reaper_payload,
                run_at=now,
                interval_seconds=reaper_interval_sec,
            )
        )
    except Exception:
        pass

