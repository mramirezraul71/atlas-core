"""Plan actions: split safe vs approval vs deferred."""
from __future__ import annotations

import os
from typing import List, Optional

from .models import ActionCandidate, ActionPlan
from .scorer import score_finding


def _approval_threshold() -> str:
    return os.getenv("GA_APPROVAL_REQUIRED_FOR", "medium").strip().lower()


def _risk_order(r: str) -> int:
    return {"low": 0, "medium": 1, "high": 2, "critical": 3}.get(r, 1)


def plan(
    findings: List,
    mode: str = "plan_only",
) -> ActionPlan:
    """Split candidates into safe / approvals / deferred by risk and mode."""
    threshold = _approval_threshold()
    thresh_val = _risk_order(threshold)
    safe: List[ActionCandidate] = []
    approvals: List[ActionCandidate] = []
    deferred: List[ActionCandidate] = []

    for f in findings:
        cand = score_finding(f)
        rv = _risk_order(cand.risk_level)
        if rv < thresh_val:
            if mode in ("controlled", "auto"):
                safe.append(cand)
            else:
                deferred.append(cand)
        else:
            approvals.append(cand)

    return ActionPlan(safe=safe, approvals=approvals, deferred=deferred)
