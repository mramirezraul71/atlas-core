"""
Workshop Jobs: ensure_workshop_jobs() para el scheduler.
Registra el job 'workshop_cycle' según variables de entorno.
"""
from __future__ import annotations

import logging
import os
from typing import Any, Dict

_log = logging.getLogger("humanoid.scheduler.workshop_jobs")


def _default_interval_seconds() -> int:
    """Intervalo por defecto del ciclo workshop (env: WORKSHOP_INTERVAL_SECONDS, default 1800 = 30 min)."""
    try:
        return max(60, int(os.getenv("WORKSHOP_INTERVAL_SECONDS", "1800") or 1800))
    except Exception:
        return 1800


def _default_mode() -> str:
    """Modo por defecto del workshop (env: WORKSHOP_DEFAULT_MODE, default 'incidents')."""
    m = (os.getenv("WORKSHOP_DEFAULT_MODE") or "incidents").strip().lower()
    return m if m in ("full", "incidents", "maintenance") else "incidents"


def _require_approval_heavy() -> bool:
    """Si las operaciones 'heavy' (full/maintenance) requieren aprobación (env: WORKSHOP_REQUIRE_APPROVAL_HEAVY)."""
    val = (os.getenv("WORKSHOP_REQUIRE_APPROVAL_HEAVY") or "true").strip().lower()
    return val in ("1", "true", "yes")


def _approval_cooldown_seconds() -> int:
    """Cooldown entre solicitudes de aprobación (env: WORKSHOP_APPROVAL_COOLDOWN_SECONDS, default 900s)."""
    try:
        return max(60, int(os.getenv("WORKSHOP_APPROVAL_COOLDOWN_SECONDS", "900") or 900))
    except Exception:
        return 900


def _workshop_enabled() -> bool:
    """Habilita o deshabilita el job del Workshop (env: WORKSHOP_ENABLED, default true)."""
    val = (os.getenv("WORKSHOP_ENABLED") or "true").strip().lower()
    return val in ("1", "true", "yes")


def ensure_workshop_jobs() -> Dict[str, Any]:
    """
    Registra o actualiza el job 'workshop_cycle' en el scheduler.
    Retorna info del job registrado o error.
    """
    if not _workshop_enabled():
        _log.info("Workshop jobs disabled (WORKSHOP_ENABLED != true)")
        return {"ok": True, "enabled": False, "reason": "WORKSHOP_ENABLED is false"}

    try:
        from modules.humanoid.scheduler.api import upsert_job, get_job
    except ImportError as e:
        _log.warning("Scheduler API not available: %s", e)
        return {"ok": False, "error": f"scheduler_api_unavailable: {e}"}

    job_id = "workshop_cycle"
    interval = _default_interval_seconds()
    mode = _default_mode()
    require_approval = _require_approval_heavy()
    cooldown = _approval_cooldown_seconds()

    payload = {
        "mode": mode,
        "limit": int(os.getenv("WORKSHOP_INCIDENT_LIMIT", "50") or 50),
        "require_approval_heavy": require_approval,
        "approval_cooldown_seconds": cooldown,
    }

    job_spec = {
        "id": job_id,
        "name": "Workshop Central Cycle",
        "kind": "workshop_cycle",
        "interval_seconds": interval,
        "enabled": True,
        "payload": payload,
        "description": f"Ciclo de reparación central: mode={mode}, interval={interval}s, approval_heavy={require_approval}",
    }

    try:
        existing = get_job(job_id)
        if existing:
            # Actualizar si cambió el payload o intervalo
            needs_update = (
                existing.get("interval_seconds") != interval
                or existing.get("payload") != payload
                or not existing.get("enabled")
            )
            if not needs_update:
                _log.debug("Workshop job already exists and is current: %s", job_id)
                return {"ok": True, "job_id": job_id, "action": "already_current", "job": existing}
    except Exception:
        pass

    try:
        result = upsert_job(job_spec)
        _log.info("Workshop job upserted: %s => %s", job_id, result)
        return {"ok": True, "job_id": job_id, "action": "upserted", "result": result}
    except Exception as e:
        _log.exception("Failed to upsert workshop job: %s", e)
        return {"ok": False, "job_id": job_id, "error": str(e)}


def disable_workshop_jobs() -> Dict[str, Any]:
    """Deshabilita el job del workshop (útil para mantenimiento manual)."""
    try:
        from modules.humanoid.scheduler.api import upsert_job, get_job

        job_id = "workshop_cycle"
        existing = get_job(job_id)
        if not existing:
            return {"ok": True, "job_id": job_id, "action": "not_found"}
        existing["enabled"] = False
        result = upsert_job(existing)
        _log.info("Workshop job disabled: %s", job_id)
        return {"ok": True, "job_id": job_id, "action": "disabled", "result": result}
    except Exception as e:
        return {"ok": False, "error": str(e)}
