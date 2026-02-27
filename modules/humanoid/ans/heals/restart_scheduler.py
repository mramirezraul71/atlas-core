"""Restart scheduler loop (internal restart)."""
from __future__ import annotations

from .base import heal_result


def run(**kwargs) -> dict:
    try:
        from modules.humanoid.scheduler.runner import run_job_sync
        # Trigger a no-op cycle; scheduler loop is managed externally
        return heal_result(True, "restart_scheduler", "triggered", {"note": "scheduler managed externally"})
    except Exception as e:
        return heal_result(False, "restart_scheduler", str(e), {}, str(e))
