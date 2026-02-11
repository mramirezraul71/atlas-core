"""Runtime scan: metrics, latencies, error-rate, scheduler health."""
from __future__ import annotations

from typing import Any, Dict, List


def scan_runtime(scope: str = "runtime", max_items: int = 30) -> Dict[str, Any]:
    """Returns {ok, findings: [{kind, path, detail, score}]} from metrics and scheduler."""
    findings: List[Dict[str, Any]] = []

    try:
        from modules.humanoid.metrics import get_metrics_store
        store = get_metrics_store()
        snap = store.snapshot()
    except Exception:
        snap = {}
    latencies = snap.get("latencies") or {}
    counters = snap.get("counters") or {}
    for name, stat in latencies.items():
        avg = stat.get("avg_ms", 0)
        if avg > 5000:
            findings.append({"kind": "high_latency", "path": name, "detail": f"avg {avg:.0f} ms", "score": min(1.0, avg / 10000), "autofix_allowed": False})
    err_count = sum(v for k, v in counters.items() if "error" in k.lower())
    if err_count > 10:
        findings.append({"kind": "error_rate", "path": "metrics", "detail": f"{err_count} errors", "score": 0.5, "autofix_allowed": False})

    try:
        from modules.humanoid.scheduler.engine import is_scheduler_running
        if not is_scheduler_running():
            findings.append({"kind": "scheduler_stopped", "path": "scheduler", "detail": "Scheduler not running", "score": 0.4, "autofix_allowed": False})
    except Exception:
        findings.append({"kind": "scheduler_unknown", "path": "scheduler", "detail": "Could not check scheduler", "score": 0.1, "autofix_allowed": False})

    return {"ok": True, "findings": findings[:max_items]}
