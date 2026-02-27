"""Governed Autonomy cycle: detect -> plan -> execute safe -> approvals -> report."""
from __future__ import annotations

import os
import time
import uuid
from typing import Any, Dict, List, Optional

from .approvals_bridge import create_approvals_batch
from .detector import detect
from .executor import execute_safe
from .models import ActionPlan, ExecutionResult, Finding
from .planner import plan
from .reporter import export_markdown, get_latest_report_path


def _ga_enabled() -> bool:
    return os.getenv("GOVERNED_AUTONOMY_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("ga", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


def _snapshot_metrics() -> Optional[Dict[str, Any]]:
    try:
        from modules.humanoid.metrics import get_metrics_store
        s = get_metrics_store().snapshot()
        latencies = (s.get("latencies") or {}).get("latencies") or {}
        counters = s.get("counters") or {}
        total = sum(v for k, v in counters.items() if k.startswith("request:") and not k.startswith("request_error:"))
        errors = sum(v for k, v in counters.items() if k.startswith("request_error:"))
        rate = errors / total if total else 0
        return {"latency": str(latencies)[:200], "error_rate": f"{rate:.2%}"}
    except Exception:
        return None


def run_cycle(
    scope: str = "all",
    mode: Optional[str] = None,
    max_findings: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Run GA cycle. scope: runtime|repo|all. mode: plan_only|controlled|auto.
    Returns {ok, data: {findings, plan, executed, approvals_created, report_path, correlation_id}, ms, error}.
    """
    t0 = time.perf_counter()
    if not _ga_enabled():
        return {"ok": False, "data": {}, "ms": int((time.perf_counter() - t0) * 1000), "error": "GOVERNED_AUTONOMY_ENABLED=false"}
    mode = mode or os.getenv("GA_MODE", "plan_only").strip()
    max_findings = max_findings if max_findings is not None else int(os.getenv("GA_MAX_FINDINGS", "10") or 10)
    correlation_id = str(uuid.uuid4())[:12]

    findings: List[Finding] = detect(scope=scope, max_findings=max_findings)
    action_plan: ActionPlan = plan(findings, mode=mode)

    executed: List[ExecutionResult] = []
    approvals_created: List[Dict[str, Any]] = []
    limit = int(os.getenv("GA_SAFE_AUTORUN_LIMIT", "2") or 2)
    strict = os.getenv("GA_STRICT_EVIDENCE", "true").strip().lower() in ("1", "true", "yes")

    if mode in ("controlled", "auto") and action_plan.safe:
        executed = execute_safe(action_plan.safe, limit=limit, strict_evidence=strict)

    if action_plan.approvals:
        approvals_created = create_approvals_batch(
            action_plan.approvals,
            plan_summary={"safe_count": len(executed), "approval_count": len(action_plan.approvals)},
            correlation_id=correlation_id,
        )

    metrics = _snapshot_metrics()
    next_action = None
    if action_plan.safe and len(executed) < len(action_plan.safe):
        next_action = f"Execute remaining {len(action_plan.safe) - len(executed)} safe actions"
    elif action_plan.approvals:
        next_action = f"Review {len(action_plan.approvals)} approval(s)"
    elif findings:
        next_action = "Monitor findings"
    else:
        next_action = "All clear"

    report_path = export_markdown(
        findings=findings,
        plan={"safe": len(action_plan.safe), "approvals": len(action_plan.approvals), "deferred": len(action_plan.deferred)},
        executed=executed,
        approvals_created=approvals_created,
        metrics=metrics,
        next_action=next_action,
    )

    ms = int((time.perf_counter() - t0) * 1000)
    _audit("ga", "run_cycle", True, {"scope": scope, "mode": mode, "findings": len(findings), "executed": len(executed)}, None, ms)
    set_last_run(time.time(), report_path)

    return {
        "ok": True,
        "data": {
            "findings": [{"source": f.source, "kind": f.kind, "path": f.path, "detail": f.detail, "score": f.score} for f in findings],
            "plan": {
                "safe": [{"action_type": c.action_type, "risk": c.risk_level, "detail": c.finding.detail} for c in action_plan.safe],
                "approvals": [{"action_type": c.action_type, "risk": c.risk_level, "detail": c.finding.detail} for c in action_plan.approvals],
                "deferred": [{"action_type": c.action_type, "detail": c.finding.detail} for c in action_plan.deferred],
            },
            "executed": [
                {"action_type": e.action_type, "ok": e.ok, "exit_code": e.exit_code, "ms": e.ms}
                for e in executed
            ],
            "approvals_created": [{"id": a.get("id"), "risk": a.get("risk"), "action": a.get("action")} for a in approvals_created],
            "report_path": report_path,
            "correlation_id": correlation_id,
        },
        "ms": ms,
        "error": None,
    }


_last_run: Optional[float] = None
_last_report: Optional[str] = None


def set_last_run(ts: float, report: Optional[str]) -> None:
    global _last_run, _last_report
    _last_run = ts
    _last_report = report


def get_status() -> Dict[str, Any]:
    """Status: last_run_ts, last_report, counters, approvals_pending_count."""
    try:
        from modules.humanoid.approvals.store import list_items
        pending = list_items(status="pending", limit=100)
        pending_count = len(pending)
    except Exception:
        pending_count = 0
    latest = get_latest_report_path()
    return {
        "last_run_ts": _last_run,
        "last_report": _last_report or latest,
        "approvals_pending_count": pending_count,
        "mode": os.getenv("GA_MODE", "plan_only"),
        "enabled": _ga_enabled(),
    }
