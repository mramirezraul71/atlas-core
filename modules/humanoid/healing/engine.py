"""Self-healing: restart scheduler, reduce concurrency; Policy + HEALING_ENABLED, max restarts."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from collections import deque
from typing import Deque, Dict, List, Optional

_log = logging.getLogger("humanoid.healing")

_restart_timestamps: Deque[float] = deque(maxlen=100)


def _healing_enabled() -> bool:
    v = os.getenv("HEALING_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _max_restarts() -> int:
    try:
        return max(1, int(os.getenv("HEALING_MAX_RESTARTS", "3") or 3))
    except (TypeError, ValueError):
        return 3


def _restart_window_seconds() -> float:
    try:
        return float(os.getenv("HEALING_RESTART_WINDOW_SECONDS", "600") or 600)
    except (TypeError, ValueError):
        return 600.0


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("healing", "system", module, action, ok, 0, error, payload, None)
    except Exception:
        pass


def _policy_allows_healing(action: str) -> bool:
    """Check policy for healing action (e.g. restart_scheduler)."""
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        ctx = ActorContext(actor="healing", role="system")
        decision = get_policy_engine().can(ctx, "healing", action, None)
        return decision.allow
    except Exception:
        return False


def _count_restarts_in_window() -> int:
    now = time.time()
    window = _restart_window_seconds()
    while _restart_timestamps and _restart_timestamps[0] < now - window:
        _restart_timestamps.popleft()
    return len(_restart_timestamps)


def can_restart_scheduler() -> bool:
    if not _healing_enabled():
        return False
    if not _policy_allows_healing("restart_scheduler"):
        return False
    if _count_restarts_in_window() >= _max_restarts():
        return False
    return True


def restart_scheduler() -> Dict[str, Any]:
    """Restart scheduler loop if allowed. Returns {ok, message}."""
    if not can_restart_scheduler():
        return {"ok": False, "message": "healing not allowed or max restarts exceeded"}
    try:
        from modules.humanoid.scheduler.engine import start_scheduler, stop_scheduler
        stop_scheduler()
        asyncio.get_event_loop().call_soon_threadsafe(lambda: start_scheduler())
        _restart_timestamps.append(time.time())
        _audit("healing", "restart_scheduler", True, {})
        return {"ok": True, "message": "scheduler restart requested"}
    except Exception as e:
        _log.exception("Healing restart_scheduler failed: %s", e)
        _audit("healing", "restart_scheduler", False, {"error": str(e)}, str(e))
        return {"ok": False, "message": str(e)}


def healing_status() -> Dict[str, Any]:
    return {
        "enabled": _healing_enabled(),
        "max_restarts": _max_restarts(),
        "restart_window_seconds": _restart_window_seconds(),
        "restarts_in_window": _count_restarts_in_window(),
        "can_restart_scheduler": can_restart_scheduler(),
    }
