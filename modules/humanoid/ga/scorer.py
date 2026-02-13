"""Risk + ROI scoring for action candidates."""
from __future__ import annotations

from typing import List

from .models import ActionCandidate, Finding

RISK_MAP = {
    "add_timeout": "low",
    "add_smoke_test": "low",
    "log_improvement": "low",
    "notify_owner": "low",
    "restart_internal_loop": "medium",
    "autofix": "low",
    "refactor_plan": "medium",
    "update_check": "low",
    "update_apply": "high",
    "restart_component": "medium",
    "tune_router": "medium",
    "remote_shell": "critical",
}

EVIDENCE_REQUIRED = {
    "add_timeout": ["path", "line", "exit_code"],
    "add_smoke_test": ["path", "exit_code"],
    "log_improvement": ["path", "exit_code"],
    "notify_owner": [],
    "restart_internal_loop": ["exit_code"],
    "autofix": ["exit_code", "changed_files"],
    "refactor_plan": [],
    "update_check": ["exit_code"],
    "update_apply": ["exit_code", "paths"],
    "restart_component": ["exit_code"],
    "tune_router": ["exit_code"],
    "remote_shell": ["exit_code", "paths"],
}


def _map_finding_to_action(f: Finding) -> tuple[str, int]:
    """Map finding kind to action_type and roi_score."""
    k = f.kind
    if k in ("latency_spike", "error_rate", "module_down"):
        return "tune_router" if "router" in f.detail.lower() else "restart_component", 60
    if k == "job_failure":
        return "restart_internal_loop", 70
    if k == "pending_update":
        return "update_check", 50
    if k in ("large_file", "todo_count"):
        return "refactor_plan", 30
    if k == "ps1_no_param":
        return "autofix", 40
    if k == "smoke_gap":
        return "add_smoke_test", 80
    if k == "timeout_missing":
        return "add_timeout", 75
    if k == "duplication":
        return "refactor_plan", 50
    if k == "repeated_errors":
        return "log_improvement", 65
    if k in ("node_offline", "gateway_error"):
        return "notify_owner", 90
    return "notify_owner", 40


def score_finding(f: Finding) -> ActionCandidate:
    """Convert Finding to ActionCandidate with risk + ROI."""
    action_type, roi = _map_finding_to_action(f)
    risk_level = RISK_MAP.get(action_type, "medium")
    evidence = EVIDENCE_REQUIRED.get(action_type, ["exit_code"])
    return ActionCandidate(
        finding=f,
        action_type=action_type,
        risk_level=risk_level,
        roi_score=min(100, roi),
        evidence_required=evidence,
        payload={"path": f.path, "kind": f.kind, "detail": f.detail, **f.meta},
    )
