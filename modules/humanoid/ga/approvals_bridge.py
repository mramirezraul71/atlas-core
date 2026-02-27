"""Bridge: create ApprovalItems from risky GA actions."""
from __future__ import annotations

import os
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Optional

from .models import ActionCandidate


def _owner_require_session_for_risk() -> str:
    """Risk levels that require owner session for approval."""
    return os.getenv("OWNER_REQUIRE_SESSION_FOR_RISK", "high,critical").strip().lower()


def create_approval(
    cand: ActionCandidate,
    plan: Dict[str, Any],
    evidence: Dict[str, Any],
    correlation_id: Optional[str] = None,
    expires_hours: int = 24,
) -> Optional[Dict[str, Any]]:
    """Create ApprovalItem for a risky action. Returns created item or None."""
    try:
        from modules.humanoid.approvals.store import create as approval_create
    except ImportError:
        return None
    risk = cand.risk_level.strip().lower()
    requires_session = risk in [x.strip() for x in _owner_require_session_for_risk().split(",") if x.strip()]
    payload = {
        "ga_action": cand.action_type,
        "finding": {
            "source": cand.finding.source,
            "kind": cand.finding.kind,
            "path": cand.finding.path,
            "detail": cand.finding.detail,
        },
        "plan": plan,
        "evidence": evidence,
        "roi_score": cand.roi_score,
    }
    if correlation_id:
        payload["correlation_id"] = correlation_id
    item = approval_create(
        action=f"ga_{cand.action_type}",
        payload=payload,
        risk=risk,
        requires_2fa=requires_session or risk in ("high", "critical"),
    )
    return item


def create_approvals_batch(
    candidates: List[ActionCandidate],
    plan_summary: Dict[str, Any],
    correlation_id: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """Create ApprovalItems for all approval-required candidates."""
    created: List[Dict[str, Any]] = []
    for c in candidates:
        item = create_approval(
            cand=c,
            plan=plan_summary,
            evidence=c.payload,
            correlation_id=correlation_id,
        )
        if item:
            created.append(item)
    return created
