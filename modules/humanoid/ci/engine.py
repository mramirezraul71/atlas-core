"""CI engine: run improve cycle (scan -> plan -> optional execute -> report)."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

from .scanner import scan_repo
from .runtime_scan import scan_runtime
from .planner import build_plan
from .executor import execute_plan
from .reporter import build_report, export_markdown
from .policy_gate import _ci_enabled, _ci_autofix_limit


def run_improve(
    scope: str = "all",
    mode: str = "plan_only",
    depth: int = 2,
    max_items: int = 10,
) -> Dict[str, Any]:
    """
    Run CI cycle. scope: repo|runtime|all. mode: plan_only|controlled|auto.
    Returns {ok, data: {findings, plan, approvals_required, auto_executed, artifacts}, ms, error}.
    """
    t0 = time.perf_counter()
    if not _ci_enabled():
        return {"ok": False, "data": {}, "ms": 0, "error": "CI_ENABLED=false"}
    max_items = max_items or int(os.getenv("CI_MAX_ITEMS", "10") or 10)
    findings: List[Dict[str, Any]] = []
    if scope in ("repo", "all"):
        r = scan_repo(scope=scope, max_items=max_items)
        findings.extend(r.get("findings") or [])
    if scope in ("runtime", "all"):
        r = scan_runtime(scope=scope, max_items=max_items)
        findings.extend(r.get("findings") or [])
    plan = build_plan(findings, scope=scope, max_items=max_items)
    auto_executed: List[Dict[str, Any]] = []
    artifacts: List[str] = []
    if mode == "auto" and (plan.get("auto_executed") or []):
        ex = execute_plan(plan, max_auto=_ci_autofix_limit())
        auto_executed = ex.get("executed") or []
        if ex.get("errors"):
            plan["auto_executed"] = [e for e in plan.get("auto_executed") or [] if not ex.get("errors")]
    report = build_report(plan, executed=auto_executed)
    export_path = export_markdown(plan, report)
    if export_path:
        artifacts.append(export_path)
    try:
        from modules.humanoid.memory_engine import ensure_thread, memory_write
        from modules.humanoid.audit import get_audit_logger
        tid = ensure_thread(None, "CI")
        memory_write(tid, "summary", {"content": f"CI cycle scope={scope} findings={len(findings)} plan_id={plan.get('plan_id')}"})
        get_audit_logger().log_event("ci", "system", "engine", "run_improve", True, int((time.perf_counter() - t0) * 1000), None, {"scope": scope, "plan_id": plan.get("plan_id")}, None)
    except Exception:
        pass
    set_improve_status({**plan, "auto_executed": auto_executed})
    ms = int((time.perf_counter() - t0) * 1000)
    return {
        "ok": True,
        "data": {
            "findings": plan.get("findings") or [],
            "plan": {"commits": plan.get("commits"), "steps": plan.get("steps")},
            "approvals_required": plan.get("approvals_required") or [],
            "auto_executed": auto_executed,
            "artifacts": artifacts,
            "report": report,
        },
        "ms": ms,
        "error": None,
    }


_last_status: Optional[Dict[str, Any]] = None


def ensure_ci_jobs() -> None:
    """Create default CI scheduler jobs if SCHED_ENABLED and CI_ENABLED and they don't exist."""
    if not _ci_enabled():
        return
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from modules.humanoid.scheduler.models import JobSpec
        from datetime import datetime, timezone
        db = get_scheduler_db()
        jobs = db.list_jobs()
        names = {j.get("name") for j in (jobs or [])}
        now = datetime.now(timezone.utc).isoformat()
        if "nightly_repo_scan" not in names:
            db.create_job(JobSpec(name="nightly_repo_scan", kind="ci_improve", payload={"scope": "repo", "mode": "plan_only", "depth": 3}, run_at=now, interval_seconds=86400))
        if "hourly_runtime_check" not in names:
            db.create_job(JobSpec(name="hourly_runtime_check", kind="ci_improve", payload={"scope": "runtime", "mode": "plan_only", "depth": 2}, run_at=now, interval_seconds=3600))
    except Exception:
        pass


def set_improve_status(data: Dict[str, Any]) -> None:
    global _last_status
    _last_status = data


def get_improve_status() -> Dict[str, Any]:
    global _last_status
    if _last_status is None:
        return {"last_cycle": None, "timestamp": None, "executed": [], "pending": []}
    return {
        "last_cycle": _last_status.get("plan_id"),
        "timestamp": _last_status.get("created_ts"),
        "executed": _last_status.get("auto_executed") or [],
        "pending": _last_status.get("approvals_required") or [],
    }


def get_last_plan() -> Optional[Dict[str, Any]]:
    """Return full last plan for apply. None if no run yet."""
    return _last_status
