"""Registra el job de ciclo del monitor de repo (actualización automática) si está habilitado."""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path


def _sched_enabled() -> bool:
    return os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parent.parent.parent.parent


def _load_repo_monitor_config() -> dict:
    """Carga config/repo_monitor.yaml para cycle.enabled e interval_seconds."""
    root = _repo_root()
    path = root / "config" / "repo_monitor.yaml"
    if not path.is_file():
        return {"cycle": {"enabled": True, "interval_seconds": 600}}
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data
    except Exception:
        return {"cycle": {"enabled": True, "interval_seconds": 600}}


def ensure_repo_monitor_jobs() -> None:
    """Crea el job repo_monitor_cycle si cycle.enabled y no existe. Actualiza intervalo si ya existe."""
    if not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        cfg = _load_repo_monitor_config()
        cycle = cfg.get("cycle") or {}
        if not cycle.get("enabled", True):
            return

        interval_sec = int(cycle.get("interval_seconds", 600) or 600)
        interval_sec = max(60, min(interval_sec, 86400))  # entre 1 min y 24 h

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=100) or []
        repo_job = next((j for j in jobs if j.get("name") == "repo_monitor_cycle"), None)

        if repo_job:
            conn = db._ensure()
            conn.execute(
                "UPDATE jobs SET interval_seconds = ?, updated_ts = ? WHERE id = ?",
                (interval_sec, datetime.now(timezone.utc).isoformat(), repo_job.get("id")),
            )
            conn.commit()
            return

        now = datetime.now(timezone.utc).isoformat()
        db.create_job(JobSpec(
            name="repo_monitor_cycle",
            kind="repo_monitor_cycle",
            payload={},
            run_at=now,
            interval_seconds=interval_sec,
        ))
    except Exception:
        pass
