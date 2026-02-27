"""Clear stale locks in scheduler/audit."""
from __future__ import annotations

import os
from .base import heal_result


def run(**kwargs) -> dict:
    try:
        if os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes"):
            from modules.humanoid.scheduler import get_scheduler_db
            db = get_scheduler_db()
            conn = db._ensure()
            conn.execute("UPDATE jobs SET lease_until = NULL WHERE lease_until < datetime('now')")
            conn.commit()
        return heal_result(True, "clear_stale_locks", "cleared", {})
    except Exception as e:
        return heal_result(False, "clear_stale_locks", str(e), {}, str(e))
