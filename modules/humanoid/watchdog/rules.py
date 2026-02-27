"""Watchdog rules: latency, error rate, scheduler alive, stale jobs, service health."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Tuple


def _float_env(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def _int_env(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def max_latency_ms() -> float:
    return _float_env("WATCHDOG_MAX_LATENCY_MS", 4000.0)


def max_error_rate() -> float:
    return _float_env("WATCHDOG_MAX_ERROR_RATE", 0.3)


def stale_job_threshold_sec() -> float:
    """Un job se considera colgado si lleva más de este tiempo en 'running'."""
    return _float_env("WATCHDOG_STALE_JOB_SEC", 60.0)


def check_latency(latencies_snapshot: Dict[str, Any], max_ms: float) -> List[Dict[str, Any]]:
    """Return list of alerts for endpoints exceeding max_ms (avg or p95)."""
    alerts = []
    latencies = (latencies_snapshot or {}).get("latencies") or latencies_snapshot
    if not isinstance(latencies, dict):
        return alerts
    targets = ["request:/llm", "request:/humanoid/plan"]
    for key in targets:
        stats = latencies.get(key)
        if not stats or not isinstance(stats, dict):
            continue
        avg = stats.get("avg_ms") or 0
        p95 = stats.get("p95_ms") or 0
        if avg > max_ms or p95 > max_ms:
            alerts.append({"rule": "latency", "key": key, "avg_ms": avg, "p95_ms": p95, "threshold_ms": max_ms})
    return alerts


def check_error_rate(counters: Dict[str, int], max_rate: float) -> List[Dict[str, Any]]:
    """Return alerts if error rate > max_rate. Expects counters like request:/path and request_error:/path."""
    alerts = []
    total = sum(v for k, v in counters.items() if k.startswith("request:") and not k.startswith("request_error:"))
    errors = sum(v for k, v in counters.items() if k.startswith("request_error:"))
    if total > 0 and (errors / total) > max_rate:
        alerts.append({"rule": "error_rate", "errors": errors, "total": total, "rate": errors / total, "threshold": max_rate})
    return alerts


def check_scheduler_alive(scheduler_running: bool) -> List[Dict[str, Any]]:
    """Alert if scheduler should be running but is not."""
    alerts = []
    if not scheduler_running:
        try:
            import os
            sched = os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
            humanoid = os.getenv("HUMANOID_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
            if sched and humanoid:
                alerts.append({"rule": "scheduler_stopped", "message": "scheduler loop is not running"})
        except Exception:
            pass
    return alerts


def check_stale_jobs() -> List[Dict[str, Any]]:
    """Detecta jobs del scheduler que llevan demasiado tiempo en status 'running'
    (lease_until pasado) y podrían estar colgados (e.g. OCR sin timeout)."""
    alerts: List[Dict[str, Any]] = []
    threshold = stale_job_threshold_sec()
    try:
        from modules.humanoid.scheduler.engine import get_scheduler_db
        db = get_scheduler_db()
        jobs = db.list_jobs()
        now_iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime())
        for j in jobs:
            status = (j.get("status") or "").lower()
            if status != "running":
                continue
            lease = j.get("lease_until") or ""
            if lease and lease < now_iso:
                # Lease expirado pero sigue en 'running' = colgado
                alerts.append({
                    "rule": "stale_job",
                    "job_id": j.get("id"),
                    "kind": j.get("kind"),
                    "lease_until": lease,
                    "message": f"Job '{j.get('name', j.get('id', '?'))}' colgado (lease expirado)",
                })
            elif not lease:
                # Sin lease pero running: potencialmente colgado
                last_run = j.get("last_run_ts") or ""
                if last_run:
                    try:
                        from datetime import datetime, timezone
                        lr = datetime.fromisoformat(last_run.replace("Z", "+00:00"))
                        elapsed = (datetime.now(timezone.utc) - lr).total_seconds()
                        if elapsed > threshold:
                            alerts.append({
                                "rule": "stale_job",
                                "job_id": j.get("id"),
                                "kind": j.get("kind"),
                                "elapsed_sec": round(elapsed, 1),
                                "message": f"Job '{j.get('name', j.get('id', '?'))}' running hace {round(elapsed)}s sin lease",
                            })
                    except Exception:
                        pass
    except Exception:
        pass
    return alerts


def check_critical_services() -> List[Dict[str, Any]]:
    """Verifica que servicios críticos (adapter en 8000, robot en 8002) respondan."""
    import urllib.request
    alerts: List[Dict[str, Any]] = []
    services = [
        ("atlas_adapter", os.getenv("ATLAS_ADAPTER_URL", "http://127.0.0.1:8000").rstrip("/") + "/health"),
        ("nexus_robot", (os.getenv("NEXUS_ROBOT_URL") or os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/") + "/api/health"),
    ]
    for name, url in services:
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=3) as r:
                if r.status >= 400:
                    alerts.append({"rule": "service_down", "service": name, "url": url, "status": r.status})
        except Exception as e:
            alerts.append({"rule": "service_down", "service": name, "url": url, "error": str(e)[:120]})
    return alerts
