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
    """Crea/actualiza jobs repo_monitor_cycle y repo_monitor_after_fix segun config."""
    if not _sched_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec

        cfg = _load_repo_monitor_config()
        db = get_scheduler_db()
        jobs = db.list_jobs(limit=100) or []
        now = datetime.now(timezone.utc).isoformat()

        # Job 1: ciclo (fetch + status) cada N segundos
        cycle = cfg.get("cycle") or {}
        if cycle.get("enabled", True):
            interval_sec = int(cycle.get("interval_seconds", 600) or 600)
            interval_sec = max(60, min(interval_sec, 86400))
            repo_job = next((j for j in jobs if j.get("name") == "repo_monitor_cycle"), None)
            if repo_job:
                conn = db._ensure()
                conn.execute(
                    "UPDATE jobs SET interval_seconds = ?, updated_ts = ? WHERE id = ?",
                    (interval_sec, now, repo_job.get("id")),
                )
                conn.commit()
                # Si quedó terminal sin next_run_ts, re-encolar.
                try:
                    status = (repo_job.get("status") or "").strip().lower()
                    next_ts = (repo_job.get("next_run_ts") or "").strip()
                    if status in ("failed", "success", "paused") and not next_ts:
                        db.set_queued(repo_job.get("id"), now)
                except Exception:
                    pass
            else:
                db.create_job(JobSpec(
                    name="repo_monitor_cycle",
                    kind="repo_monitor_cycle",
                    payload={},
                    run_at=now,
                    interval_seconds=interval_sec,
                ))

        # Job 2: after-fix (commit + push) cada N segundos si esta habilitado
        af = cfg.get("after_fix") or {}
        auto_interval = int(af.get("auto_schedule_interval_seconds", 0) or 0)
        if af.get("enabled", True) and auto_interval > 0:
            auto_interval = max(300, min(auto_interval, 86400))  # entre 5 min y 24 h
            after_job = next((j for j in jobs if j.get("name") == "repo_monitor_after_fix"), None)
            if after_job:
                conn = db._ensure()
                conn.execute(
                    "UPDATE jobs SET interval_seconds = ?, updated_ts = ? WHERE id = ?",
                    (auto_interval, now, after_job.get("id")),
                )
                conn.commit()
                try:
                    status = (after_job.get("status") or "").strip().lower()
                    next_ts = (after_job.get("next_run_ts") or "").strip()
                    if status in ("failed", "success", "paused") and not next_ts:
                        db.set_queued(after_job.get("id"), now)
                except Exception:
                    pass
            else:
                db.create_job(JobSpec(
                    name="repo_monitor_after_fix",
                    kind="repo_monitor_after_fix",
                    payload={},
                    run_at=now,
                    interval_seconds=auto_interval,
                ))
    except Exception:
        pass
