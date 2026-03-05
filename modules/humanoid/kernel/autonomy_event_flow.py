"""Autonomous Event Bus flow for ATLAS.

Registers default subscriptions on the kernel EventBus and triggers
automatic actions for core operational events.
"""
from __future__ import annotations

import logging
import os
from typing import Any, Dict

from .event_bus import EventBus

_log = logging.getLogger("atlas.kernel.autonomy_event_flow")
_BOOTSTRAPPED = False


def _env_bool(name: str, default: bool) -> bool:
    raw = (os.getenv(name) or ("true" if default else "false")).strip().lower()
    return raw in ("1", "true", "yes", "y", "on")


def _append_log(message: str, ok: bool = True, source: str = "event_bus") -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log

        append_evolution_log(message=message, ok=ok, source=source)
    except Exception as exc:
        _log.debug("append_evolution_log unavailable: %s", exc)


def _notify_ops(message: str, level: str = "info", data: Dict[str, Any] | None = None) -> None:
    try:
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        ops_emit("event_bus", message, level=level, data=data or {})
    except Exception as exc:
        _log.debug("ops_bus emit unavailable: %s", exc)


def _dispatch_pot_for_event(event_type: str, pot_id: str | None, context: Dict[str, Any]) -> None:
    try:
        from modules.humanoid.quality.dispatcher import dispatch_event

        dispatch_event(event_type=event_type, pot_id=pot_id, context=context)
    except Exception as exc:
        _log.warning("dispatch_event failed for %s: %s", event_type, exc)


def _on_memory_update(payload: Dict[str, Any] | None = None, **kwargs: Any) -> None:
    data = dict(payload or {})
    data.update(kwargs)
    summary = str(data.get("summary") or data.get("message") or "memory updated")
    _append_log(f"[EVENT] memory_update: {summary}", ok=True, source="event_bus")
    _notify_ops(
        "memory_update registrado",
        level="low",
        data={"event": "memory_update", "summary": summary},
    )


def _on_git_change(payload: Dict[str, Any] | None = None, **kwargs: Any) -> None:
    data = dict(payload or {})
    data.update(kwargs)
    changed = data.get("changed_files") or []
    if isinstance(changed, str):
        changed = [changed]
    count = len(changed) if isinstance(changed, list) else int(data.get("count") or 0)
    _append_log(f"[EVENT] git_change: {count} archivo(s) detectados", ok=True, source="event_bus")
    _dispatch_pot_for_event("git_change", "git_safe_sync", data)


def _on_api_error(payload: Dict[str, Any] | None = None, **kwargs: Any) -> None:
    data = dict(payload or {})
    data.update(kwargs)
    endpoint = str(data.get("endpoint") or "unknown")
    error = str(data.get("error") or data.get("message") or "api_error")
    _append_log(f"[EVENT] api_error @{endpoint}: {error[:180]}", ok=False, source="event_bus")
    _notify_ops(
        f"api_error detectado en {endpoint}",
        level="high",
        data={"event": "api_error", "endpoint": endpoint, "error": error[:300]},
    )
    _dispatch_pot_for_event("api_error", "api_repair", data)


def _on_task_complete(payload: Dict[str, Any] | None = None, **kwargs: Any) -> None:
    data = dict(payload or {})
    data.update(kwargs)
    task_id = str(data.get("task_id") or data.get("id") or "unknown")
    status = str(data.get("status") or "completed")
    _append_log(f"[EVENT] task_complete #{task_id} ({status})", ok=True, source="event_bus")
    _notify_ops(
        f"Tarea completada: {task_id}",
        level="info",
        data={"event": "task_complete", "task_id": task_id, "status": status},
    )
    _dispatch_pot_for_event("task_complete", "notification_broadcast", data)


def bootstrap_autonomy_event_flow(bus: EventBus) -> Dict[str, Any]:
    """Register default autonomous event subscriptions once per process."""
    global _BOOTSTRAPPED

    if _BOOTSTRAPPED:
        return {"ok": True, "bootstrapped": True, "already_bootstrapped": True}

    if not _env_bool("ATLAS_EVENT_FLOW_ENABLED", True):
        return {"ok": True, "bootstrapped": False, "disabled": True}

    bus.subscribe("memory_update", _on_memory_update, priority=5)
    bus.subscribe("git_change", _on_git_change, priority=7)
    bus.subscribe("api_error", _on_api_error, priority=9)
    bus.subscribe("task_complete", _on_task_complete, priority=4)

    _BOOTSTRAPPED = True
    _log.info("Autonomy Event Flow bootstrapped with 4 subscriptions")
    return {
        "ok": True,
        "bootstrapped": True,
        "topics": ["memory_update", "git_change", "api_error", "task_complete"],
    }

