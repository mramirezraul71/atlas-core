"""Base event handlers for hybrid scheduler/watchdog operational events.

These handlers are intentionally non-intrusive: they observe and log events so
the current system behavior stays unchanged while exposing future integration
points for more autonomous reactions.
"""
from __future__ import annotations

import json
import hashlib
import logging
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any, Dict, Optional

from modules.humanoid.kernel.event_bus import EventBus

_log = logging.getLogger("atlas.core.event_handlers")
_BOOTSTRAPPED = False
_OBS_LOCK = Lock()
_DEDUP_LOCK = Lock()
_EVENT_DEDUP_CACHE: Dict[str, float] = {}


def _repo_root() -> Path:
    root = (
        os.getenv("ATLAS_REPO_PATH")
        or os.getenv("ATLAS_PUSH_ROOT")
        or os.getenv("ATLAS_ROOT")
        or ""
    ).strip()
    if root:
        return Path(root).resolve()
    here = Path(__file__).resolve()
    return here.parents[3]


def _event_log_path() -> Path:
    path = _repo_root() / "logs" / "hybrid_event_handlers.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _append_jsonl(record: Dict[str, Any]) -> None:
    line = json.dumps(record, ensure_ascii=True, default=str)
    with _OBS_LOCK:
        with _event_log_path().open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")


def _emit_ops_message(message: str, *, level: str, data: Dict[str, Any]) -> None:
    try:
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        ops_emit("event_bus", message, level=level, data=data)
    except Exception as exc:
        _log.debug("ops_bus emit unavailable: %s", exc)


def _append_evolution_note(message: str, *, ok: bool, source: str) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log

        append_evolution_log(message=message, ok=ok, source=source)
    except Exception as exc:
        _log.debug("append_evolution_log unavailable: %s", exc)


def _record_handler_result(
    event_type: str,
    payload: Dict[str, Any],
    *,
    action: str,
    ok: bool,
    source: str,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    record = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "event_type": event_type,
        "source": source,
        "action": action,
        "ok": bool(ok),
        "payload": payload,
        "details": details or {},
    }
    _append_jsonl(record)
    return record


def _dedupe_window_seconds() -> float:
    try:
        return max(1.0, float(os.getenv("HYBRID_EVENT_DEDUP_SEC", "8") or "8"))
    except (TypeError, ValueError):
        return 8.0


def _event_fingerprint(event_type: str, payload: Dict[str, Any]) -> str:
    data = {
        "event_type": event_type,
        "rule": payload.get("rule"),
        "component": payload.get("component"),
        "change_type": payload.get("change_type"),
        "source": payload.get("source"),
        "error": str(payload.get("error") or "")[:120],
    }
    raw = json.dumps(data, sort_keys=True, ensure_ascii=True, default=str)
    return hashlib.sha1(raw.encode("utf-8", "ignore")).hexdigest()


def _is_duplicated_watchdog_event(event_type: str, payload: Dict[str, Any]) -> tuple[bool, str]:
    fingerprint = _event_fingerprint(event_type, payload)
    now = time.time()
    dedupe_window = _dedupe_window_seconds()
    with _DEDUP_LOCK:
        for key, ts in list(_EVENT_DEDUP_CACHE.items()):
            if now - ts > dedupe_window:
                _EVENT_DEDUP_CACHE.pop(key, None)
        last_seen = _EVENT_DEDUP_CACHE.get(fingerprint)
        if last_seen and (now - last_seen) < dedupe_window:
            return True, fingerprint
        _EVENT_DEDUP_CACHE[fingerprint] = now
    return False, fingerprint


def _queue_supervisor_review(
    event_type: str,
    payload: Dict[str, Any],
    *,
    recommended_action: str,
    source: str,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    duplicated, fingerprint = _is_duplicated_watchdog_event(event_type, payload)
    if duplicated:
        return {
            "ok": True,
            "queued": False,
            "deduped": True,
            "fingerprint": fingerprint,
            "reason": "duplicate_in_dedupe_window",
        }

    try:
        from modules.humanoid.supervisor.autoprogrammer import queue_supervisor_review

        queued = queue_supervisor_review(
            event_type,
            payload=payload,
            recommended_action=recommended_action,
            source=source,
            details=details or {},
        )
        return {
            "ok": bool(queued.get("ok", True)),
            "queued": bool(queued.get("queued", True)),
            "deduped": False,
            "fingerprint": fingerprint,
            "result": queued,
        }
    except Exception as exc:
        return {
            "ok": False,
            "queued": False,
            "deduped": False,
            "fingerprint": fingerprint,
            "error": str(exc),
        }


def _future_memory_integration(payload: Dict[str, Any]) -> None:
    """Reserved hook for ATLAS persistent memory ingestion.

    Future integration point:
    - persist operational traces into ATLAS episodic/semantic memory
    - enrich post-mortem context for recovery and learning
    """
    return None


def _future_supervisor_autofix(payload: Dict[str, Any]) -> None:
    """Reserved hook for supervisor autofix orchestration.

    Future integration point:
    - trigger safe remediation plans
    - delegate recovery actions to supervisor modules
    """
    return None


def _future_intelligent_scheduler(payload: Dict[str, Any]) -> None:
    """Reserved hook for intelligent scheduling optimization.

    Future integration point:
    - reprioritize jobs based on observed state transitions
    - adapt cadence/backoff using operational telemetry
    """
    return None


def _merge_payload(payload: Optional[Dict[str, Any]], kwargs: Dict[str, Any]) -> Dict[str, Any]:
    data = dict(payload or {})
    data.update(kwargs)
    return data


def _on_scheduler_job_new(payload: Optional[Dict[str, Any]] = None, **kwargs: Any) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    _log.info(
        "Event scheduler.job.new job_id=%s kind=%s source=%s",
        data.get("job_id") or data.get("id"),
        data.get("kind"),
        source,
    )
    _record_handler_result(
        "scheduler.job.new",
        data,
        action="observe_job_new",
        ok=True,
        source=source,
    )
    _future_memory_integration(data)


def _on_scheduler_job_state_changed(
    payload: Optional[Dict[str, Any]] = None, **kwargs: Any
) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    _log.info(
        "Event scheduler.job.state_changed job_id=%s %s->%s source=%s",
        data.get("job_id") or data.get("id"),
        data.get("old_status"),
        data.get("new_status"),
        source,
    )
    _record_handler_result(
        "scheduler.job.state_changed",
        data,
        action="observe_state_transition",
        ok=True,
        source=source,
        details={
            "old_status": data.get("old_status"),
            "new_status": data.get("new_status"),
        },
    )
    _future_intelligent_scheduler(data)


def _on_scheduler_job_completed(payload: Optional[Dict[str, Any]] = None, **kwargs: Any) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    job_id = str(data.get("job_id") or data.get("id") or "unknown")
    result = {
        "job_id": job_id,
        "ok": bool(data.get("ok", False)),
        "ms": data.get("ms"),
        "status": data.get("status"),
        "kind": data.get("kind"),
    }
    _log.info(
        "Event scheduler.job.completed job_id=%s ok=%s ms=%s",
        job_id,
        result["ok"],
        result["ms"],
    )
    _append_evolution_note(
        f"[EVENT] scheduler.job.completed #{job_id} ok={result['ok']} ms={result['ms']}",
        ok=result["ok"],
        source="event_bus",
    )
    _record_handler_result(
        "scheduler.job.completed",
        data,
        action="record_job_result",
        ok=True,
        source=source,
        details=result,
    )
    _future_memory_integration(data)
    _future_intelligent_scheduler(data)


def _on_watchdog_logs_error(payload: Optional[Dict[str, Any]] = None, **kwargs: Any) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    rule = str(data.get("rule") or "unknown")
    recommended_action = "investigate_error_and_prepare_patch"
    _log.warning(
        "Event watchdog.logs.error rule=%s source=%s",
        rule,
        source,
    )
    _append_evolution_note(
        f"[EVENT] watchdog.logs.error rule={rule}",
        ok=False,
        source="event_bus",
    )
    supervisor = _queue_supervisor_review(
        "watchdog.logs.error",
        data,
        recommended_action=recommended_action,
        source=source,
        details={"rule": rule},
    )
    _record_handler_result(
        "watchdog.logs.error",
        data,
        action="queue_supervisor_review",
        ok=bool(supervisor.get("ok", False)),
        source=source,
        details={
            "handler_action": "queue_supervisor_review",
            "recommended_action": recommended_action,
            "rule": rule,
            "deduped": bool(supervisor.get("deduped", False)),
            "supervisor": supervisor,
        },
    )
    _future_memory_integration(data)


def _on_watchdog_process_down(payload: Optional[Dict[str, Any]] = None, **kwargs: Any) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    component = str(data.get("component") or "unknown")
    recommended_action = "stabilize_process_and_queue_supervisor_review"
    diagnostic: Dict[str, Any] = {"component": component, "rule": data.get("rule")}

    if component == "scheduler":
        try:
            from modules.humanoid.scheduler.engine import (
                get_running_count,
                get_scheduler_db,
                is_scheduler_running,
            )

            diagnostic.update(
                {
                    "scheduler_running": bool(is_scheduler_running()),
                    "running_count": int(get_running_count()),
                    "recent_jobs": (get_scheduler_db().list_jobs(limit=5) or []),
                }
            )
        except Exception as exc:
            diagnostic["diagnostic_error"] = str(exc)

    _log.warning(
        "Event watchdog.process.down component=%s rule=%s",
        component,
        data.get("rule"),
    )
    _append_evolution_note(
        f"[EVENT] watchdog.process.down component={component}",
        ok=False,
        source="event_bus",
    )
    _emit_ops_message(
        f"Proceso degradado detectado: {component}",
        level="high",
        data={"event": "watchdog.process.down", **diagnostic},
    )
    supervisor = _queue_supervisor_review(
        "watchdog.process.down",
        data,
        recommended_action=recommended_action,
        source=source,
        details=diagnostic,
    )
    _record_handler_result(
        "watchdog.process.down",
        data,
        action="queue_supervisor_review",
        ok=bool(supervisor.get("ok", False)),
        source=source,
        details={
            **diagnostic,
            "handler_action": "queue_supervisor_review",
            "recommended_action": recommended_action,
            "deduped": bool(supervisor.get("deduped", False)),
            "supervisor": supervisor,
        },
    )
    _future_supervisor_autofix(data)


def _on_watchdog_change_detected(
    payload: Optional[Dict[str, Any]] = None, **kwargs: Any
) -> None:
    data = _merge_payload(payload, kwargs)
    source = str(data.get("source") or "unknown")
    recommended_action = "queue_supervisor_review_for_change"
    hint = {
        "recommended_action": recommended_action,
        "change_type": data.get("change_type"),
        "rule": data.get("rule"),
        "component": data.get("component"),
    }
    _log.info(
        "Event watchdog.change.detected change=%s rule=%s",
        data.get("change_type"),
        data.get("rule"),
    )
    supervisor = _queue_supervisor_review(
        "watchdog.change.detected",
        data,
        recommended_action=recommended_action,
        source=source,
        details=hint,
    )
    _record_handler_result(
        "watchdog.change.detected",
        data,
        action="queue_supervisor_review",
        ok=bool(supervisor.get("ok", False)),
        source=source,
        details={
            **hint,
            "handler_action": "queue_supervisor_review",
            "deduped": bool(supervisor.get("deduped", False)),
            "supervisor": supervisor,
        },
    )
    _emit_ops_message(
        f"Watchdog change detectado: {data.get('rule')}",
        level="med",
        data={"event": "watchdog.change.detected", **hint},
    )
    _future_memory_integration(data)
    _future_supervisor_autofix(data)


def register_default_event_handlers(bus: Optional[EventBus] = None) -> Dict[str, Any]:
    """Register base handlers once per process."""
    global _BOOTSTRAPPED
    if _BOOTSTRAPPED:
        return {"ok": True, "bootstrapped": True, "already_bootstrapped": True}

    if bus is None:
        from .event_bus import get_event_bus

        bus = get_event_bus()

    bus.subscribe("scheduler.job.new", _on_scheduler_job_new, priority=3)
    bus.subscribe(
        "scheduler.job.state_changed",
        _on_scheduler_job_state_changed,
        priority=4,
    )
    bus.subscribe("scheduler.job.completed", _on_scheduler_job_completed, priority=5)
    bus.subscribe("watchdog.logs.error", _on_watchdog_logs_error, priority=6)
    bus.subscribe("watchdog.process.down", _on_watchdog_process_down, priority=7)
    bus.subscribe("watchdog.change.detected", _on_watchdog_change_detected, priority=2)

    _BOOTSTRAPPED = True
    return {
        "ok": True,
        "bootstrapped": True,
        "topics": [
            "scheduler.job.new",
            "scheduler.job.state_changed",
            "scheduler.job.completed",
            "watchdog.logs.error",
            "watchdog.process.down",
            "watchdog.change.detected",
        ],
    }


__all__ = ["register_default_event_handlers"]
