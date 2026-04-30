"""Scheduler engine: background asyncio loop, due jobs, max concurrency, audit."""
from __future__ import annotations

import asyncio
import json
import logging
import os
from datetime import datetime, timezone
from importlib import import_module
from pathlib import Path
from typing import Any, Dict, Optional, Set

from modules.humanoid.core.event_bus import publish_event
from modules.humanoid.core.event_handlers import register_default_event_handlers
from .db import SchedulerDB
from .locking import recover_stale_locks, try_acquire_lease

_log = logging.getLogger("humanoid.scheduler.engine")

_sched_db: Optional[SchedulerDB] = None
_loop_task: Optional[asyncio.Task] = None
_owner_loop: Optional[asyncio.AbstractEventLoop] = None
_running = 0
_cancel_requested: Dict[str, bool] = {}
_executor: Optional[Any] = None
_known_job_ids: Set[str] = set()
_last_job_statuses: Dict[str, str] = {}


def _append_scheduler_trace(event: str, payload: Dict[str, Any]) -> None:
    try:
        base = Path(os.getenv("ATLAS_BASE", Path(__file__).resolve().parents[3]))
        trace_dir = base / "state"
        trace_dir.mkdir(parents=True, exist_ok=True)
        trace_path = trace_dir / "atlas_scheduler_trace.jsonl"
        row = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "source": "engine",
            "event": event,
            **(payload or {}),
        }
        with trace_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    except Exception:
        pass


def _get_run_job_sync():
    """Resolve runner lazily so the scheduler uses the live runner implementation."""
    module = import_module("modules.humanoid.scheduler.runner")
    return module.run_job_sync


def _sched_enabled() -> bool:
    v = os.getenv("SCHED_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _tick_seconds() -> float:
    return float(os.getenv("SCHED_TICK_SECONDS", "1") or 1)


def _max_concurrency() -> int:
    return max(1, int(os.getenv("SCHED_MAX_CONCURRENCY", "1") or 1))


def _default_retries() -> int:
    return max(0, int(os.getenv("SCHED_DEFAULT_RETRIES", "3") or 3))


def _backoff_seconds() -> int:
    return max(0, int(os.getenv("SCHED_BACKOFF_SECONDS", "5") or 5))


def get_scheduler_db() -> SchedulerDB:
    global _sched_db
    if _sched_db is None:
        _sched_db = SchedulerDB()
    return _sched_db


def _audit(
    module: str,
    action: str,
    ok: bool,
    payload: Optional[Dict] = None,
    error: Optional[str] = None,
    ms: int = 0,
) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger

        get_audit_logger().log_event(
            "scheduler", "system", module, action, ok, ms, error, payload, None
        )
    except Exception:
        pass


def _normalize_status(value: Any) -> str:
    return str(value or "").strip().lower() or "unknown"


def _track_job_snapshot(job: Dict[str, Any]) -> None:
    jid = str(job.get("id") or "")
    if not jid:
        return
    _known_job_ids.add(jid)
    _last_job_statuses[jid] = _normalize_status(job.get("status"))


def _emit_scheduler_event(topic: str, job: Dict[str, Any], **extra: Any) -> None:
    payload = {
        "job_id": job.get("id"),
        "name": job.get("name"),
        "kind": job.get("kind"),
        "status": job.get("status"),
        "retries": job.get("retries"),
        "next_run_ts": job.get("next_run_ts"),
        "last_run_ts": job.get("last_run_ts"),
    }
    payload.update(extra)
    publish_event(topic, payload)


def _prime_scheduler_event_state(db: SchedulerDB) -> None:
    global _known_job_ids, _last_job_statuses
    jobs = db.list_jobs(limit=1000) or []
    _known_job_ids = {str(job.get("id") or "") for job in jobs if job.get("id")}
    _last_job_statuses = {
        str(job.get("id")): _normalize_status(job.get("status"))
        for job in jobs
        if job.get("id")
    }


def _reconcile_scheduler_event_state(db: SchedulerDB) -> None:
    """Polling fallback: detect jobs created/changed outside this engine."""
    global _known_job_ids, _last_job_statuses

    jobs = db.list_jobs(limit=1000) or []
    current_ids: Set[str] = set()
    current_statuses: Dict[str, str] = {}

    for job in jobs:
        jid = str(job.get("id") or "")
        if not jid:
            continue
        status = _normalize_status(job.get("status"))
        current_ids.add(jid)
        current_statuses[jid] = status

        if jid not in _known_job_ids:
            _emit_scheduler_event("scheduler.job.new", job, source="poll_fallback")

        previous_status = _last_job_statuses.get(jid)
        if previous_status and previous_status != status:
            _emit_scheduler_event(
                "scheduler.job.state_changed",
                job,
                old_status=previous_status,
                new_status=status,
                source="poll_fallback",
            )
            if status == "success":
                _emit_scheduler_event(
                    "scheduler.job.completed",
                    job,
                    ok=True,
                    source="poll_fallback",
                )

    _known_job_ids = current_ids
    _last_job_statuses = current_statuses


async def _run_one_job(job: Dict[str, Any]) -> None:
    global _running
    jid = job.get("id", "")
    kind = job.get("kind", "")
    db = get_scheduler_db()
    loop = asyncio.get_event_loop()
    run_job_sync = _get_run_job_sync()
    _append_scheduler_trace(
        "run_one_job_start",
        {
            "job_id": jid,
            "kind_raw": job.get("kind"),
            "kind_type": type(job.get("kind")).__name__,
            "runner_module": getattr(run_job_sync, "__module__", None),
            "runner_name": getattr(run_job_sync, "__name__", None),
            "runner_file": getattr(getattr(run_job_sync, "__code__", None), "co_filename", None),
        },
    )
    if _executor is None:
        result = run_job_sync(job)
    else:
        result = await loop.run_in_executor(_executor, run_job_sync, job)

    now = datetime.now(timezone.utc).isoformat()
    ok = result.get("ok", False)
    ms = result.get("ms", 0)
    err = result.get("error")
    result_json = result.get("result_json")
    _append_scheduler_trace(
        "run_one_job_after_runner",
        {
            "job_id": jid,
            "kind_raw": job.get("kind"),
            "ok": ok,
            "ms": ms,
            "error": err,
            "result_json_head": str(result_json)[:500] if result_json is not None else None,
        },
    )

    db.insert_run(jid, job.get("last_run_ts") or now, now, ok, ms, result_json, err)
    _append_scheduler_trace(
        "run_one_job_inserted_run",
        {
            "job_id": jid,
            "kind_raw": job.get("kind"),
            "ok": ok,
            "ms": ms,
        },
    )
    _audit(
        "scheduler", "job_run_end", ok, {"job_id": jid, "kind": kind, "ms": ms}, err, ms
    )

    retries = job.get("retries", 0)
    max_retries = job.get("max_retries", _default_retries())
    backoff = job.get("backoff_seconds", _backoff_seconds())
    interval = job.get("interval_seconds")

    if ok:
        next_ts = None
        if interval:
            from datetime import timedelta

            next_dt = datetime.fromisoformat(now.replace("Z", "+00:00")) + timedelta(
                seconds=interval
            )
            next_ts = next_dt.isoformat()
        db.set_finished(jid, True, None, next_ts, retries)
        finished_job = db.get_job(jid) or dict(job, status="success", next_run_ts=next_ts)
        _emit_scheduler_event(
            "scheduler.job.state_changed",
            finished_job,
            old_status="running",
            new_status="success",
            ok=True,
            ms=ms,
            source="event_first",
        )
        _emit_scheduler_event(
            "scheduler.job.completed",
            finished_job,
            ok=True,
            ms=ms,
            source="event_first",
        )
        _track_job_snapshot(finished_job)
        if interval and next_ts:
            db.set_queued(jid, next_ts)
            requeued_job = db.get_job(jid) or dict(
                finished_job,
                status="queued",
                next_run_ts=next_ts,
            )
            _emit_scheduler_event(
                "scheduler.job.state_changed",
                requeued_job,
                old_status="success",
                new_status="queued",
                recurring=True,
                source="event_first",
            )
            _track_job_snapshot(requeued_job)
        _running -= 1
        return

    retries += 1
    if retries >= max_retries:
        next_ts = None
        db.set_finished(jid, False, err, next_ts, retries)
        failed_job = db.get_job(jid) or dict(
            job,
            status="failed",
            last_error=err,
            retries=retries,
            next_run_ts=next_ts,
        )
        _emit_scheduler_event(
            "scheduler.job.state_changed",
            failed_job,
            old_status="running",
            new_status="failed",
            ok=False,
            error=err,
            source="event_first",
        )
        _track_job_snapshot(failed_job)
        _audit(
            "scheduler", "job_failed", False, {"job_id": jid, "retries": retries}, err
        )
    else:
        from datetime import timedelta

        # Progressive backoff: backoff_seconds * (retries)
        delay = backoff * retries
        next_dt = datetime.fromisoformat(now.replace("Z", "+00:00")) + timedelta(
            seconds=delay
        )
        next_ts = next_dt.isoformat()
        db.set_finished(jid, False, err, next_ts, retries)
        failed_job = db.get_job(jid) or dict(
            job,
            status="failed",
            last_error=err,
            retries=retries,
            next_run_ts=next_ts,
        )
        _emit_scheduler_event(
            "scheduler.job.state_changed",
            failed_job,
            old_status="running",
            new_status="failed",
            ok=False,
            error=err,
            source="event_first",
        )
        _track_job_snapshot(failed_job)
        db.set_queued(jid, next_ts)
        requeued_job = db.get_job(jid) or dict(
            failed_job,
            status="queued",
            next_run_ts=next_ts,
        )
        _emit_scheduler_event(
            "scheduler.job.state_changed",
            requeued_job,
            old_status="failed",
            new_status="queued",
            retry=retries,
            source="event_first",
        )
        _track_job_snapshot(requeued_job)
        _audit(
            "scheduler",
            "job_retry",
            True,
            {
                "job_id": jid,
                "retry": retries,
                "next_run": next_ts,
                "backoff_seconds": delay,
            },
        )
    _running -= 1


async def _tick() -> None:
    global _running
    db = get_scheduler_db()
    now = datetime.now(timezone.utc).isoformat()
    recover_stale_locks(db)
    _reconcile_scheduler_event_state(db)
    limit = max(1, _max_concurrency() - _running)
    if limit <= 0:
        return
    due = db.due_jobs(now, limit)
    for job in due:
        jid = job.get("id", "")
        if _cancel_requested.pop(jid, False):
            db.set_paused(jid)
            paused_job = db.get_job(jid) or dict(job, status="paused")
            _emit_scheduler_event(
                "scheduler.job.state_changed",
                paused_job,
                old_status=_normalize_status(job.get("status")),
                new_status="paused",
                source="event_first",
            )
            _track_job_snapshot(paused_job)
            continue
        if not try_acquire_lease(db, jid):
            continue
        _running += 1
        running_job = db.get_job(jid) or dict(job, status="running")
        _emit_scheduler_event(
            "scheduler.job.state_changed",
            running_job,
            old_status=_normalize_status(job.get("status")),
            new_status="running",
            source="event_first",
        )
        _track_job_snapshot(running_job)
        _audit(
            "scheduler", "job_run_start", True, {"job_id": jid, "kind": job.get("kind")}
        )
        asyncio.create_task(_run_one_job(dict(job, last_run_ts=now)))


async def _loop() -> None:
    while _sched_enabled():
        try:
            await _tick()
        except Exception as e:
            _log.exception("Scheduler tick error: %s", e)
            _audit("scheduler", "tick_error", False, {"error": str(e)})
        await asyncio.sleep(_tick_seconds())


def start_scheduler(executor: Optional[Any] = None) -> Optional[asyncio.Task]:
    """Start background scheduler loop. Returns the asyncio Task or None if disabled."""
    global _loop_task, _executor, _owner_loop
    if not _sched_enabled():
        _log.info("Scheduler disabled (SCHED_ENABLED=false)")
        return None
    register_default_event_handlers()
    _prime_scheduler_event_state(get_scheduler_db())
    _executor = executor
    _owner_loop = asyncio.get_event_loop()
    if _loop_task is not None and not _loop_task.done():
        return _loop_task
    _loop_task = _owner_loop.create_task(_loop())
    _log.info("Scheduler loop started")
    _audit("scheduler", "start", True, {})
    return _loop_task


def stop_scheduler() -> None:
    global _loop_task
    if _loop_task and not _loop_task.done():
        _loop_task.cancel()
    _loop_task = None
    _audit("scheduler", "stop", True, {})


def request_scheduler_restart() -> bool:
    """Request a scheduler restart on the owning event loop if available."""
    global _owner_loop, _executor

    def _restart_on_owner() -> None:
        stop_scheduler()
        start_scheduler(executor=_executor)

    if _owner_loop is not None and _owner_loop.is_running():
        _owner_loop.call_soon_threadsafe(_restart_on_owner)
        return True
    try:
        loop = asyncio.get_event_loop()
        loop.call_soon(_restart_on_owner)
        return True
    except Exception:
        return False


def is_scheduler_running() -> bool:
    return _loop_task is not None and not _loop_task.done()


def request_cancel(job_id: str) -> None:
    _cancel_requested[job_id] = True


def get_running_count() -> int:
    return _running
