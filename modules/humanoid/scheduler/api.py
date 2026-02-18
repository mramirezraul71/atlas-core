"""
Scheduler API (compat): helpers para jobs por nombre.

Nota: El SchedulerDB usa `id` UUID interno. Algunos módulos legacy (p.ej. workshop_jobs)
usan "id" como *nombre* del job. Esta capa mantiene compatibilidad sin cambiar el schema.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, Optional


def get_job(job_name: str) -> Optional[Dict[str, Any]]:
    """Obtiene un job por su nombre lógico (columna jobs.name)."""
    try:
        from modules.humanoid.scheduler import get_scheduler_db

        db = get_scheduler_db()
        jobs = db.list_jobs(limit=500) or []
        return next((j for j in jobs if (j.get("name") or "") == job_name), None)
    except Exception:
        return None


def upsert_job(spec: Dict[str, Any]) -> Dict[str, Any]:
    """
    Crea o actualiza un job por nombre lógico.

    Convención legacy:
    - spec["id"] se interpreta como job_name (columna jobs.name)
    - spec["payload"] se serializa a payload_json
    """
    from modules.humanoid.scheduler import get_scheduler_db
    from modules.humanoid.scheduler.models import JobSpec

    db = get_scheduler_db()
    now = datetime.now(timezone.utc).isoformat()

    job_name = (spec.get("id") or spec.get("name") or "").strip()
    if not job_name:
        return {"ok": False, "error": "missing job name (id/name)"}

    kind = (spec.get("kind") or "custom").strip()
    interval = spec.get("interval_seconds")
    try:
        interval = int(interval) if interval is not None else None
    except Exception:
        interval = None

    enabled = bool(spec.get("enabled", True))
    payload = spec.get("payload") or {}

    existing = get_job(job_name)
    if not existing:
        created = db.create_job(
            JobSpec(
                name=job_name,
                kind=kind,
                payload=payload,
                run_at=now,
                interval_seconds=interval,
            )
        )
        return {"ok": True, "action": "created", "job": created}

    # Update in-place
    conn = db._ensure()
    conn.execute(
        "UPDATE jobs SET kind = ?, payload_json = ?, interval_seconds = ?, enabled = ?, updated_ts = ? WHERE id = ?",
        (
            kind,
            json.dumps(payload),
            interval,
            1 if enabled else 0,
            now,
            existing.get("id"),
        ),
    )
    conn.commit()

    # Si se habilitó y está terminal, re-encolar.
    try:
        status = (existing.get("status") or "").strip().lower()
        next_ts = (existing.get("next_run_ts") or "").strip()
        if enabled and status in ("failed", "success", "paused") and not next_ts:
            db.set_queued(existing.get("id"), now)
    except Exception:
        pass

    return {"ok": True, "action": "updated", "job": get_job(job_name) or existing}


__all__ = ["get_job", "upsert_job"]

