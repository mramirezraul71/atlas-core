"""Scheduler engine: background asyncio loop, due jobs, max concurrency, audit."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from .db import SchedulerDB
from .locking import recover_stale_locks, try_acquire_lease
from .runner import run_job_sync

_log = logging.getLogger("humanoid.scheduler.engine")

_sched_db: Optional[SchedulerDB] = None
_loop_task: Optional[asyncio.Task] = None
_running = 0
_cancel_requested: Dict[str, bool] = {}
_executor: Optional[Any] = None


def _sched_enabled() -> bool:
    v = os.getenv("SCHED_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _tick_seconds() -> float:
    return float(os.getenv("SCHED_TICK_SECONDS", "2") or 2)


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


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("scheduler", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


async def _run_one_job(job: Dict[str, Any]) -> None:
    global _running
    jid = job.get("id", "")
    kind = job.get("kind", "")
    db = get_scheduler_db()
    loop = asyncio.get_event_loop()
    if _executor is None:
        result = run_job_sync(job)
    else:
        result = await loop.run_in_executor(_executor, run_job_sync, job)

    now = datetime.now(timezone.utc).isoformat()
    ok = result.get("ok", False)
    ms = result.get("ms", 0)
    err = result.get("error")
    result_json = result.get("result_json")

    db.insert_run(jid, job.get("last_run_ts") or now, now, ok, ms, result_json, err)
    _audit("scheduler", "job_run_end", ok, {"job_id": jid, "kind": kind, "ms": ms}, err, ms)

    retries = job.get("retries", 0)
    max_retries = job.get("max_retries", _default_retries())
    backoff = job.get("backoff_seconds", _backoff_seconds())
    interval = job.get("interval_seconds")

    if ok:
        next_ts = None
        if interval:
            from datetime import timedelta
            next_dt = datetime.fromisoformat(now.replace("Z", "+00:00")) + timedelta(seconds=interval)
            next_ts = next_dt.isoformat()
        db.set_finished(jid, True, None, next_ts, retries)
        if interval and next_ts:
            db.set_queued(jid, next_ts)
        _running -= 1
        return

    retries += 1
    if retries >= max_retries:
        next_ts = None
        db.set_finished(jid, False, err, next_ts, retries)
        _audit("scheduler", "job_failed", False, {"job_id": jid, "retries": retries}, err)
    else:
        from datetime import timedelta
        # Progressive backoff: backoff_seconds * (retries)
        delay = backoff * retries
        next_dt = datetime.fromisoformat(now.replace("Z", "+00:00")) + timedelta(seconds=delay)
        next_ts = next_dt.isoformat()
        db.set_finished(jid, False, err, next_ts, retries)
        db.set_queued(jid, next_ts)
        _audit("scheduler", "job_retry", True, {"job_id": jid, "retry": retries, "next_run": next_ts, "backoff_seconds": delay})
    _running -= 1


async def _tick() -> None:
    global _running
    db = get_scheduler_db()
    now = datetime.now(timezone.utc).isoformat()
    recover_stale_locks(db)
    limit = max(1, _max_concurrency() - _running)
    if limit <= 0:
        return
    due = db.due_jobs(now, limit)
    for job in due:
        jid = job.get("id", "")
        if _cancel_requested.pop(jid, False):
            db.set_paused(jid)
            continue
        if not try_acquire_lease(db, jid):
            continue
        _running += 1
        _audit("scheduler", "job_run_start", True, {"job_id": jid, "kind": job.get("kind")})
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
    global _loop_task, _executor
    if not _sched_enabled():
        _log.info("Scheduler disabled (SCHED_ENABLED=false)")
        return None
    _executor = executor
    if _loop_task is not None and not _loop_task.done():
        return _loop_task
    loop = asyncio.get_event_loop()
    _loop_task = loop.create_task(_loop())
    _log.info("Scheduler loop started")
    _audit("scheduler", "start", True, {})
    return _loop_task


def stop_scheduler() -> None:
    global _loop_task
    if _loop_task and not _loop_task.done():
        _loop_task.cancel()
    _loop_task = None
    _audit("scheduler", "stop", True, {})


def is_scheduler_running() -> bool:
    return _loop_task is not None and not _loop_task.done()


def request_cancel(job_id: str) -> None:
    _cancel_requested[job_id] = True


def get_running_count() -> int:
    return _running
