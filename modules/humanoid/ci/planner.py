"""Convert findings into plan: commits, steps, approvals_required."""
from __future__ import annotations

import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List

from .policy_gate import SAFE_AUTOFIX_KINDS, can_autofix, _ci_autofix_limit
from .scorer import score_findings


def build_plan(
    findings: List[Dict[str, Any]],
    scope: str,
    max_items: int = 10,
) -> Dict[str, Any]:
    """Build ImprovementPlan-like dict: plan_id, findings (scored), commits, steps, approvals_required, auto_executed."""
    plan_id = str(uuid.uuid4())[:8]
    scored = score_findings(findings)[:max_items]
    commits: List[Dict[str, Any]] = []
    steps: List[Dict[str, Any]] = []
    approvals_required: List[Dict[str, Any]] = []
    auto_executed: List[Dict[str, Any]] = []
    limit = _ci_autofix_limit()
    for i, f in enumerate(scored):
        kind = f.get("kind") or ""
        path = f.get("path") or ""
        item_id = f"{plan_id}_{i}"
        if kind == "ps1_no_param" and can_autofix("add_param_defaults") and len(auto_executed) < limit:
            auto_executed.append({"item_id": item_id, "action": "add_param_defaults", "path": path, "finding": f})
        else:
            approvals_required.append({"item_id": item_id, "finding": f, "reason": "requires_review"})
        steps.append({"id": item_id, "description": f"{kind}: {path}", "detail": f.get("detail", "")})
    if auto_executed:
        commits.append({"message": f"chore(ci): apply safe autofix ({len(auto_executed)} items)", "items": [a["item_id"] for a in auto_executed]})
    return {
        "plan_id": plan_id,
        "scope": scope,
        "findings": scored,
        "commits": commits,
        "steps": steps,
        "approvals_required": approvals_required,
        "auto_executed": auto_executed,
        "artifacts": [],
        "created_ts": datetime.now(timezone.utc).isoformat(),
    }
