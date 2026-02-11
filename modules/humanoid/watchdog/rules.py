"""Watchdog rules: latency, error rate, scheduler alive."""
from __future__ import annotations

import os
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
