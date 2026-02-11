"""Job locking with lease: acquire, release, stale recovery."""
from __future__ import annotations

import logging
import os
from datetime import datetime, timezone
from typing import Optional

from .db import SchedulerDB

_log = logging.getLogger("humanoid.scheduler.locking")


def _lease_seconds() -> int:
    return max(5, int(os.getenv("SCHED_LEASE_SECONDS", "30") or 30))


def _stale_recovery_enabled() -> bool:
    v = os.getenv("SCHED_STALE_RECOVERY", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def try_acquire_lease(db: SchedulerDB, job_id: str) -> bool:
    """
    Try to set job to running and set lease_until = now + lease_seconds.
    Returns True if this worker acquired the lease (row was updated).
    Only updates if status is queued/failed, or running with expired lease.
    """
    now = datetime.now(timezone.utc)
    from datetime import timedelta
    lease_until = (now + timedelta(seconds=_lease_seconds())).isoformat()
    now_iso = now.isoformat()
    conn = db._ensure()
    # Condition: (status IN ('queued','failed')) OR (status='running' AND (lease_until IS NULL OR lease_until < now))
    cur = conn.execute(
        """UPDATE jobs SET status = 'running', lease_until = ?, updated_ts = ? WHERE id = ?
           AND (enabled = 1) AND (status IN ('queued', 'failed')
               OR (status = 'running' AND (lease_until IS NULL OR lease_until < ?)))""",
        (lease_until, now_iso, job_id, now_iso),
    )
    conn.commit()
    return cur.rowcount > 0


def release_lease(db: SchedulerDB, job_id: str) -> None:
    """Clear lease_until for job (e.g. when finished)."""
    now = datetime.now(timezone.utc).isoformat()
    db._ensure().execute("UPDATE jobs SET lease_until = NULL, updated_ts = ? WHERE id = ?", (now, job_id))
    db._ensure().commit()


def recover_stale_locks(db: Optional[SchedulerDB] = None) -> int:
    """
    Set jobs that are running but have lease_until < now back to queued.
    Returns number of jobs recovered.
    """
    if not _stale_recovery_enabled():
        return 0
    if db is None:
        from .engine import get_scheduler_db
        db = get_scheduler_db()
    now = datetime.now(timezone.utc).isoformat()
    conn = db._ensure()
    cur = conn.execute(
        """UPDATE jobs SET status = 'queued', lease_until = NULL, updated_ts = ? WHERE status = 'running' AND lease_until IS NOT NULL AND lease_until < ?""",
        (now, now),
    )
    conn.commit()
    n = cur.rowcount
    if n > 0:
        _log.warning("Stale lock recovery: %d job(s) requeued", n)
    return n
