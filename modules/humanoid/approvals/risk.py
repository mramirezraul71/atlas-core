"""Risk scoring: low|medium|high|critical. requires_confirm, requires_2fa."""
from __future__ import annotations

import os
from typing import Any, Dict, Optional


def risk_level(action: str, payload: Optional[Dict[str, Any]] = None) -> str:
    """Return risk: critical | high | medium | low."""
    action_lower = (action or "").strip().lower()
    if "deploy_apply" in action_lower or ("apply" in action_lower and "update" in action_lower):
        return "critical"
    if "delete" in action_lower or "rollback" in action_lower:
        return "critical"
    if "apply" in action_lower or "promote" in action_lower:
        return "high"
    if "execute" in action_lower or "run" in action_lower:
        return "medium"
    return "low"


def _2fa_for() -> str:
    return (os.getenv("OWNER_2FA_REQUIRED_FOR", "critical") or "critical").strip().lower()


def requires_2fa_for_risk(risk: str) -> bool:
    threshold = _2fa_for()
    ord_map = {"low": 0, "medium": 1, "high": 2, "critical": 3}
    return ord_map.get((risk or "").strip().lower(), 0) >= ord_map.get(threshold, 3)


def requires_confirm_for_risk(risk: str) -> bool:
    """High and critical require double confirmation."""
    r = (risk or "").strip().lower()
    return r in ("high", "critical")
