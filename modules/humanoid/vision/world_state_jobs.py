"""Job del scheduler: WorldState tick (captura + OCR + métricas + persistencia).

Objetivo: que ATLAS mantenga una representación reciente del entorno (visual) incluso sin dashboard.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")


def _world_state_enabled() -> bool:
    return os.getenv("WORLD_STATE_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")


def _interval_seconds() -> int:
    raw = int(os.getenv("WORLD_STATE_INTERVAL_SECONDS", "30") or 30)
    return max(10, min(raw, 3600))


def ensure_world_state_jobs() -> None:
    """Create/update job `world_state_tick`."""
    if not _sched_enabled() or not _world_state_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=300) or []
        now = datetime.now(timezone.utc).isoformat()
        interval = _interval_seconds()

        job = next((j for j in jobs if j.get("name") == "world_state_tick"), None)
        if job:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, updated_ts = ? WHERE id = ?",
                (interval, now, job.get("id")),
            )
            conn.commit()
        else:
            db.create_job(
                JobSpec(
                    name="world_state_tick",
                    kind="world_state_tick",
                    payload={"include_ocr_items": False, "use_llm_vision": False},
                    run_at=now,
                    interval_seconds=interval,
                )
            )
    except Exception:
        # best-effort
        pass

