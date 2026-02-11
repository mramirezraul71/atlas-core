"""Extended health check: LLM latency, scheduler, memory, DB integrity, port, uptime. Score 0-100."""
from __future__ import annotations

import os
import time
import urllib.request
import urllib.error
from typing import Any, Dict, Optional

HEALTH_TIMEOUT_SEC = 10
_APP_START_TIME: Optional[float] = None


def _set_app_start_time(t: Optional[float] = None) -> None:
    global _APP_START_TIME
    _APP_START_TIME = t or time.perf_counter()


def _get_uptime_sec() -> float:
    if _APP_START_TIME is None:
        return 0.0
    return time.perf_counter() - _APP_START_TIME


def _check_llm_latency_avg() -> Dict[str, Any]:
    """LLM latency average from metrics store."""
    try:
        from modules.humanoid.metrics import get_metrics_store
        snap = get_metrics_store().snapshot()
        lat = snap.get("latencies") or {}
        llm = lat.get("llm") or lat.get("llm_request") or {}
        avg = llm.get("avg_ms")
        if avg is not None:
            return {"ok": True, "avg_ms": avg, "message": f"avg {avg:.0f}ms"}
        return {"ok": True, "avg_ms": None, "message": "no samples"}
    except Exception as e:
        return {"ok": False, "avg_ms": None, "message": str(e)}


def _check_scheduler_running() -> Dict[str, Any]:
    """Scheduler enabled and DB reachable."""
    try:
        if os.getenv("SCHED_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
            return {"ok": True, "running": False, "message": "scheduler disabled"}
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        _ = db.list_jobs(limit=1)
        return {"ok": True, "running": True, "message": "ok"}
    except Exception as e:
        return {"ok": False, "running": False, "message": str(e)}


def _check_memory_writable() -> Dict[str, Any]:
    """Memory DB writable (quick connect + SELECT)."""
    try:
        if os.getenv("MEMORY_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
            return {"ok": True, "writable": False, "message": "memory disabled"}
        from modules.humanoid.memory_engine import db as mem_db
        conn = mem_db._ensure()
        conn.execute("SELECT 1")
        conn.commit()
        return {"ok": True, "writable": True, "message": "ok"}
    except Exception as e:
        return {"ok": False, "writable": False, "message": str(e)}


def _check_db_integrity() -> Dict[str, Any]:
    """SQLite PRAGMA quick_check on scheduler DB. Skipped if scheduler disabled."""
    try:
        if os.getenv("SCHED_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
            return {"ok": True, "skipped": True, "message": "scheduler disabled"}
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        conn = db._ensure()
        row = conn.execute("PRAGMA quick_check").fetchone()
        ok = row and row[0] == "ok"
        return {"ok": ok, "message": row[0] if row else "unknown"}
    except Exception as e:
        return {"ok": False, "message": str(e)}


def _check_port_active(base_url: str) -> Dict[str, Any]:
    """Port/instance responding (GET /status)."""
    url = f"{base_url.rstrip('/')}/status"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=HEALTH_TIMEOUT_SEC) as r:
            if r.status == 200:
                return {"ok": True, "message": "ok"}
            return {"ok": False, "message": f"status {r.status}"}
    except urllib.error.URLError as e:
        return {"ok": False, "message": str(e.reason) if getattr(e, "reason", None) else str(e)}
    except Exception as e:
        return {"ok": False, "message": str(e)}


def run_health(base_url: Optional[str] = None) -> Dict[str, Any]:
    """
    Run extended health checks. If base_url is None or same as current instance, run in-process.
    Otherwise HTTP GET base_url/health (so the other instance computes and returns).
    """
    if base_url:
        url = f"{base_url.rstrip('/')}/health"
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=HEALTH_TIMEOUT_SEC) as r:
                import json
                return json.loads(r.read().decode())
        except Exception as e:
            return {"ok": False, "score": 0, "error": str(e), "checks": {}}

    # In-process
    llm = _check_llm_latency_avg()
    sched = _check_scheduler_running()
    mem = _check_memory_writable()
    db = _check_db_integrity()
    port = {"ok": True, "message": "current process"}
    uptime_sec = _get_uptime_sec()

    checks = {
        "llm_latency_avg": llm,
        "scheduler_running": sched,
        "memory_writable": mem,
        "db_integrity": db,
        "port_active": port,
        "uptime_sec": round(uptime_sec, 1),
    }
    score = health_score(checks)
    return {
        "ok": score >= 60,
        "score": score,
        "checks": checks,
        "error": None if score >= 60 else "health score < 60",
    }


def health_score(checks: Dict[str, Any]) -> int:
    """Compute 0-100 score from checks dict. Skipped checks are not penalized."""
    if not checks:
        return 0
    ok_count = 0
    n = 0
    for key, val in checks.items():
        if key == "uptime_sec":
            ok_count += 1 if isinstance(val, (int, float)) and val >= 0 else 0
            n += 1
        elif isinstance(val, dict):
            if val.get("skipped"):
                continue  # don't count skipped in denominator
            n += 1
            if val.get("ok"):
                ok_count += 1
    if n == 0:
        return 100  # all skipped => healthy
    return min(100, int(round(100.0 * ok_count / n)))
