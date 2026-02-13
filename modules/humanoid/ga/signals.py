"""Signal collectors: watchdog, scheduler, update, ci, memory, gateway/cluster."""
from __future__ import annotations

import os
from typing import Any, Dict, List

from .models import Finding


def _safe(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except Exception:
        return []


def collect_watchdog_signals() -> List[Finding]:
    """Watchdog: latency spikes, error-rate, module down."""
    findings: List[Finding] = []
    try:
        from modules.humanoid.watchdog.engine import get_last_alerts
        alerts = _safe(get_last_alerts) or []
        for a in alerts[:5]:
            kind = a.get("rule", "alert")
            path = a.get("key", a.get("message", "runtime"))
            detail = a.get("detail") or a.get("message") or str(a)
            score = float(a.get("score", 0.5))
            findings.append(Finding(
                source="watchdog",
                kind=kind,
                path=str(path),
                detail=detail,
                score=score,
                meta=a,
            ))
    except ImportError:
        pass
    return findings


def collect_scheduler_signals() -> List[Finding]:
    """Scheduler: job failures/retries."""
    findings: List[Finding] = []
    try:
        from modules.humanoid.scheduler.engine import get_scheduler_db
        db = get_scheduler_db()
        jobs = _safe(db.list_jobs) or []
        seen: set = set()
        for j in jobs[:20]:
            jid = j.get("id") or j.get("job_id")
            if not jid or jid in seen:
                continue
            seen.add(jid)
            runs = _safe(db.get_runs, jid, 5) or []
            for r in runs:
                if r.get("ok") is False:
                    findings.append(Finding(
                        source="scheduler",
                        kind="job_failure",
                        path=str(jid),
                        detail=r.get("error", "Job failed"),
                        score=0.6,
                        meta={"job_id": jid, "job_name": j.get("name"), **r},
                    ))
                    break
            if len(findings) >= 5:
                break
    except ImportError:
        pass
    return findings


def collect_update_signals() -> List[Finding]:
    """Update: pending updates detected."""
    findings: List[Finding] = []
    try:
        from modules.humanoid.update.update_engine import check, update_enabled
        if not _safe(update_enabled):
            return []
        result = _safe(check) or {}
        data = result.get("data") or result
        if result.get("ok") and data.get("has_update"):
            findings.append(Finding(
                source="update",
                kind="pending_update",
                path="repo",
                detail=data.get("message", "Updates available"),
                score=0.4,
                meta=result,
            ))
    except ImportError:
        pass
    return findings


def collect_ci_signals(scope: str) -> List[Finding]:
    """CI: repo findings (duplication, timeouts, smoke gaps)."""
    findings: List[Finding] = []
    if scope not in ("repo", "all"):
        return []
    try:
        from modules.humanoid.ci.scanner import scan_repo
        data = _safe(scan_repo, scope=scope, max_items=10) or {}
        for f in data.get("findings", [])[:10]:
            findings.append(Finding(
                source="ci",
                kind=f.get("kind", "finding"),
                path=f.get("path", "repo"),
                detail=f.get("detail", ""),
                score=float(f.get("score", 0.3)),
                meta=f,
            ))
    except ImportError:
        pass
    return findings


def collect_memory_signals() -> List[Finding]:
    """Memory: repeated errors (same stacktrace N times in 24h)."""
    findings: List[Finding] = []
    try:
        from modules.humanoid.memory.store import query_recent_errors
        errors = _safe(query_recent_errors) or []
        if errors:
            findings.append(Finding(
                source="memory",
                kind="repeated_errors",
                path="memory",
                detail=f"{len(errors)} recent errors",
                score=min(0.8, 0.3 + len(errors) * 0.1),
                meta={"count": len(errors), "samples": errors[:3]},
            ))
    except ImportError:
        pass
    return findings


def collect_gateway_signals() -> List[Finding]:
    """Gateway/Cluster: node offline/degraded."""
    findings: List[Finding] = []
    try:
        from modules.humanoid.gateway.store import build_gateway_status
        st = _safe(build_gateway_status) or {}
        if st.get("error"):
            findings.append(Finding(
                source="gateway",
                kind="gateway_error",
                path="cluster",
                detail=st.get("error", "Gateway degraded"),
                score=0.6,
                meta=st,
            ))
    except ImportError:
        pass
    return findings


def collect_all_signals(scope: str = "all") -> List[Finding]:
    """Collect all available signals; fallback gracefully on missing modules."""
    out: List[Finding] = []
    out.extend(collect_watchdog_signals())
    out.extend(collect_scheduler_signals())
    out.extend(collect_update_signals())
    out.extend(collect_ci_signals(scope))
    out.extend(collect_memory_signals())
    out.extend(collect_gateway_signals())
    return out
