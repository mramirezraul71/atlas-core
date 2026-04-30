"""Watchdog engine: periodic check of metrics + scheduler, publish events + audit.

Self-healing: solo reinicia loops internos (p. ej. scheduler). Nunca reinicia el proceso
o sistema completo sin aprobación explícita (policy/flag)."""
from __future__ import annotations

import asyncio
import json
import logging
import os
from typing import Any, Dict, List, Optional, Set

from modules.humanoid.core.event_bus import publish_event
from modules.humanoid.core.event_handlers import register_default_event_handlers
from .rules import (check_error_rate, check_latency, check_scheduler_alive,
                    max_error_rate, max_latency_ms)

_log = logging.getLogger("humanoid.watchdog")

_watchdog_task: Optional[asyncio.Task] = None
_last_alerts: List[Dict[str, Any]] = []
_last_alert_keys: Set[str] = set()


def _watchdog_enabled() -> bool:
    v = os.getenv("WATCHDOG_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _tick_seconds() -> float:
    try:
        return float(os.getenv("WATCHDOG_TICK_SECONDS", "1") or 1)
    except (TypeError, ValueError):
        return 1.0


def _audit(
    module: str,
    action: str,
    ok: bool,
    payload: Optional[Dict] = None,
    error: Optional[str] = None,
) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger

        get_audit_logger().log_event(
            "watchdog", "system", module, action, ok, 0, error, payload, None
        )
    except Exception:
        pass


def _publish(topic: str, payload: Dict[str, Any]) -> None:
    try:
        from modules.humanoid import get_humanoid_kernel

        k = get_humanoid_kernel()
        k.dispatch(topic, payload)
    except Exception:
        pass


def _alert_key(alert: Dict[str, Any]) -> str:
    try:
        return json.dumps(alert, sort_keys=True, default=str)
    except Exception:
        return repr(sorted((alert or {}).items()))


def _emit_watchdog_event(topic: str, payload: Optional[Dict[str, Any]] = None, **extra: Any) -> None:
    data = dict(payload or {})
    data.update(extra)
    publish_event(topic, data)


def _publish_watchdog_events(alerts: List[Dict[str, Any]]) -> None:
    """Event-first publication with polling reconciliation by alert snapshot."""
    global _last_alert_keys

    current_keys: Set[str] = set()

    for alert in alerts:
        key = _alert_key(alert)
        current_keys.add(key)
        if key in _last_alert_keys:
            continue

        rule = str(alert.get("rule") or "unknown")
        _emit_watchdog_event(
            "watchdog.change.detected",
            alert,
            change_type="alert_opened",
            source="event_first",
        )

        if rule in ("error_rate", "watchdog_error"):
            _emit_watchdog_event("watchdog.logs.error", alert, source="event_first")
        elif rule == "scheduler_stopped":
            _emit_watchdog_event(
                "watchdog.process.down",
                alert,
                component="scheduler",
                source="event_first",
            )
        else:
            _emit_watchdog_event(
                "watchdog.change.detected",
                alert,
                change_type="relevant_change",
                source="event_first",
            )

    for resolved_key in (_last_alert_keys - current_keys):
        _emit_watchdog_event(
            "watchdog.change.detected",
            {
                "rule": "watchdog_alert_resolved",
                "alert_key": resolved_key,
            },
            change_type="alert_resolved",
            source="poll_fallback",
        )

    _last_alert_keys = current_keys


async def _tick() -> None:
    global _last_alerts
    alerts: List[Dict[str, Any]] = []
    try:
        store = None
        try:
            from modules.humanoid.metrics import get_metrics_store

            store = get_metrics_store()
        except Exception:
            pass
        snapshot = store.snapshot() if store else {}
        latencies = snapshot.get("latencies") or {}
        counters = snapshot.get("counters") or {}

        alerts.extend(check_latency(latencies, max_latency_ms()))
        alerts.extend(check_error_rate(counters, max_error_rate()))

        scheduler_running = False
        try:
            from modules.humanoid.scheduler.engine import is_scheduler_running

            scheduler_running = is_scheduler_running()
        except Exception:
            pass
        alerts.extend(check_scheduler_alive(scheduler_running))

        if alerts:
            _last_alerts = alerts
            _audit(
                "watchdog",
                "alerts",
                False,
                {"alerts": alerts},
                f"{len(alerts)} alert(s)",
            )
            _publish("watchdog.alerts", {"alerts": alerts})
            _publish_watchdog_events(alerts)
            _log.warning("Watchdog alerts: %s", alerts)
            # Self-healing: if scheduler stopped, try to restart (respects healing limit)
            for a in alerts:
                if a.get("rule") == "scheduler_stopped":
                    try:
                        from modules.humanoid.healing import restart_scheduler

                        r = restart_scheduler()
                        if r.get("ok"):
                            _audit("watchdog", "healing_restart_scheduler", True, r)
                            _log.info(
                                "Watchdog triggered scheduler restart: %s",
                                r.get("message"),
                            )
                    except Exception:
                        pass
                    break
        else:
            _last_alerts = []
            _publish_watchdog_events([])
    except Exception as e:
        _log.exception("Watchdog tick error: %s", e)
        _audit("watchdog", "tick_error", False, {"error": str(e)})
        _last_alerts = [{"rule": "watchdog_error", "error": str(e)}]
        _publish_watchdog_events(_last_alerts)


async def _loop() -> None:
    while _watchdog_enabled():
        await _tick()
        await asyncio.sleep(_tick_seconds())


def start_watchdog() -> Optional[asyncio.Task]:
    if not _watchdog_enabled():
        _log.info("Watchdog disabled (WATCHDOG_ENABLED=false)")
        return None
    register_default_event_handlers()
    global _watchdog_task
    if _watchdog_task is not None and not _watchdog_task.done():
        return _watchdog_task
    loop = asyncio.get_event_loop()
    _watchdog_task = loop.create_task(_loop())
    _log.info("Watchdog loop started")
    _audit("watchdog", "start", True, {})
    return _watchdog_task


def stop_watchdog() -> None:
    global _watchdog_task
    if _watchdog_task and not _watchdog_task.done():
        _watchdog_task.cancel()
    _watchdog_task = None


def get_last_alerts() -> List[Dict[str, Any]]:
    return list(_last_alerts)


def watchdog_status() -> Dict[str, Any]:
    return {
        "enabled": _watchdog_enabled(),
        "running": _watchdog_task is not None and not _watchdog_task.done(),
        "tick_seconds": _tick_seconds(),
        "last_alerts": get_last_alerts(),
    }
