"""Scheduler health check."""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
import os
import urllib.request
import json

from modules.humanoid.scheduler.db import SchedulerDB


STALE_FAILED_DAYS = 14


def _http_json(url: str, timeout: float = 5.0) -> dict | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def _db_jobs() -> list[dict]:
    try:
        return list(SchedulerDB().list_jobs(limit=1000) or [])
    except Exception:
        return []


def _latest_run(job_id: str) -> dict | None:
    try:
        runs = SchedulerDB().get_runs(job_id, limit=1) or []
        return runs[0] if runs else None
    except Exception:
        return None


def _is_semantic_state_desync(job: dict) -> bool:
    if not isinstance(job, dict):
        return False
    if str(job.get("kind") or "").strip().lower() != "nervous_cycle":
        return False
    if str(job.get("status") or "").strip().lower() != "failed":
        return False
    if job.get("last_error"):
        return False
    run = _latest_run(str(job.get("id") or ""))
    if not isinstance(run, dict):
        return False
    if run.get("error"):
        return False
    result = run.get("result") or {}
    return isinstance(result, dict) and "score" in result and "points" in result


def _parse_iso(ts: object) -> datetime | None:
    raw = str(ts or "").strip()
    if not raw:
        return None
    try:
        if raw.endswith("Z"):
            raw = raw[:-1] + "+00:00"
        return datetime.fromisoformat(raw)
    except Exception:
        return None


def _is_stale_failed_job(job: dict) -> bool:
    if not isinstance(job, dict):
        return False
    if str(job.get("kind") or "").strip().lower() == "nervous_cycle":
        return False
    if str(job.get("status") or "").strip().lower() != "failed":
        return False
    if job.get("next_run_ts"):
        return False
    ts = _parse_iso(job.get("updated_ts")) or _parse_iso(job.get("last_run_ts"))
    if ts is None:
        return False
    if ts.tzinfo is None:
        ts = ts.replace(tzinfo=timezone.utc)
    age = datetime.now(timezone.utc) - ts.astimezone(timezone.utc)
    return age >= timedelta(days=STALE_FAILED_DAYS)


def run() -> dict:
    if os.getenv("SCHED_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
        return {
            "ok": True,
            "check_id": "scheduler_health",
            "message": "scheduler disabled",
            "details": {},
            "severity": "low",
        }
    try:
        status = _http_json("http://127.0.0.1:8791/scheduler/jobs", timeout=5.0)
        if status is None:
            jobs = _db_jobs()
            if not jobs:
                return {
                    "ok": False,
                    "check_id": "scheduler_health",
                    "message": "scheduler endpoint unavailable",
                    "details": {"url": "http://127.0.0.1:8791/scheduler/jobs", "fallback": "db_empty"},
                    "severity": "med",
                    "suggested_heals": ["restart_scheduler"],
                }
            source = "db_fallback"
        else:
            jobs = (status.get("data") or []) if isinstance(status, dict) else []
            source = "http"
        failed_enabled = [
            job
            for job in jobs
            if isinstance(job, dict)
            and bool(job.get("enabled", True))
            and str(job.get("status") or "").lower() == "failed"
        ]
        semantic_desync = [job for job in failed_enabled if _is_semantic_state_desync(job)]
        stale_failed = [job for job in failed_enabled if _is_stale_failed_job(job)]
        actionable_failed = [
            job
            for job in failed_enabled
            if not _is_stale_failed_job(job) and not _is_semantic_state_desync(job)
        ]
        stuck = [
            job for job in actionable_failed if not (job.get("next_run_ts"))
        ]
        if actionable_failed:
            primary = actionable_failed[0]
            return {
                "ok": False,
                "check_id": "scheduler_health",
                "message": f"{len(actionable_failed)} scheduler jobs failed",
                "details": {
                    "jobs_count": len(jobs),
                    "failed_count": len(actionable_failed),
                    "stale_failed_ignored": len(stale_failed),
                    "state_desync_ignored": len(semantic_desync),
                    "stuck_count": len(stuck),
                    "source": source,
                    "top_failed_job": {
                        "id": primary.get("id"),
                        "name": primary.get("name"),
                        "kind": primary.get("kind"),
                        "retries": primary.get("retries"),
                        "next_run_ts": primary.get("next_run_ts"),
                        "last_error": primary.get("last_error"),
                    },
                },
                "severity": "high" if stuck else "med",
                "suggested_heals": ["restart_scheduler"],
            }
        return {
            "ok": True,
            "check_id": "scheduler_health",
            "message": "ok" if not stale_failed else "ok (stale failed jobs ignored)",
            "details": {
                "jobs_count": len(jobs),
                "stale_failed_ignored": len(stale_failed),
                "state_desync_ignored": len(semantic_desync),
                "source": source,
            },
            "severity": "low",
        }
    except Exception as e:
        return {
            "ok": False,
            "check_id": "scheduler_health",
            "message": str(e),
            "details": {"error": str(e)},
            "severity": "med",
            "suggested_heals": ["restart_scheduler"],
        }
