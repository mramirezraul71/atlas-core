"""Repo hygiene scheduler job: evita acumulación de logs/snapshots en Git.

Diseño:
- Job periódico (por defecto cada 6h) ejecuta `scripts/repo_hygiene.py --scan`.
- Si `REPO_HYGIENE_AUTO=true`, el job puede ejecutar `--auto` (fix+commit+push).
"""

from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")


def _repo_hygiene_enabled() -> bool:
    return os.getenv("REPO_HYGIENE_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")


def _interval_seconds() -> int:
    # 6h default, clamp [10m..24h]
    raw = int(os.getenv("REPO_HYGIENE_INTERVAL_SECONDS", str(6 * 3600)) or (6 * 3600))
    return max(600, min(raw, 86400))


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parent.parent.parent.parent


def ensure_repo_hygiene_jobs() -> None:
    """Create/update repo_hygiene_cycle job."""
    if not _sched_enabled() or not _repo_hygiene_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=200) or []
        now = datetime.now(timezone.utc).isoformat()
        interval = _interval_seconds()

        job = next((j for j in jobs if j.get("name") == "repo_hygiene_cycle"), None)
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
                    name="repo_hygiene_cycle",
                    kind="repo_hygiene_cycle",
                    payload={"repo_root": str(_repo_root())},
                    run_at=now,
                    interval_seconds=interval,
                )
            )
    except Exception:
        pass

