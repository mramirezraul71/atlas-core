"""Policy gate: determine if an action requires approval (not safe autofix / read-only)."""
from __future__ import annotations

from typing import Any, Dict, Optional

# Actions that do NOT require approval (safe or read-only)
SAFE_OR_READ_ACTIONS = frozenset({
    "read", "status", "recall", "export", "list", "check", "plan_only",
    "memory_read", "memory_export", "ci_autofix",  # ci_autofix is policy-gated elsewhere
})


def requires_approval(action: str, payload: Optional[Dict[str, Any]] = None) -> bool:
    """True if this action must go to the approval queue (not safe/read)."""
    action_lower = (action or "").strip().lower()
    if action_lower in SAFE_OR_READ_ACTIONS:
        return False
    if action_lower in ("execute", "apply", "run", "update_apply", "promote", "run_now"):
        return True
    if "apply" in action_lower or "execute" in action_lower or "delete" in action_lower:
        return True
    return False


def risk_level(action: str, payload: Optional[Dict[str, Any]] = None) -> str:
    """Return risk: high | medium | low."""
    action_lower = (action or "").strip().lower()
    if "delete" in action_lower or "rollback" in action_lower or "apply" in action_lower and "update" in action_lower:
        return "high"
    if "execute" in action_lower or "run" in action_lower:
        return "medium"
    return "low"
