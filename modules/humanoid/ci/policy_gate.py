"""Policy gate: what CI can auto-execute vs requires approval."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

SAFE_AUTOFIX_KINDS = frozenset({
    "add_timeout", "add_param_defaults", "add_gitignore", "improve_error_message",
    "add_smoke_check", "minor_format", "add_typing",
})


def _ci_enabled() -> bool:
    v = os.getenv("CI_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _ci_autofix_allowed() -> bool:
    v = os.getenv("CI_ALLOWED_AUTOFIX", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _ci_autofix_limit() -> int:
    try:
        return max(0, int(os.getenv("CI_AUTOFIX_LIMIT", "2") or 2))
    except (TypeError, ValueError):
        return 2


def can_autofix(action_kind: str, actor: Any = None) -> bool:
    """True only if policy allows ci_autofix and action is in SAFE_AUTOFIX_KINDS."""
    if action_kind not in SAFE_AUTOFIX_KINDS:
        return False
    if not _ci_autofix_allowed():
        return False
    try:
        from modules.humanoid.policy import get_policy_engine, ActorContext
        ctx = actor or ActorContext(actor="ci", role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))
        d = get_policy_engine().can(ctx, "ci", "ci_autofix")
        return d.allow
    except Exception:
        return False


def classify_items(
    findings: List[Dict[str, Any]],
    proposals: List[Dict[str, Any]],
    max_auto: int,
) -> Dict[str, Any]:
    """Split into auto_executed (allowed + safe) and approvals_required."""
    auto: List[Dict[str, Any]] = []
    required: List[Dict[str, Any]] = []
    for p in proposals[: max_auto * 2]:
        kind = p.get("action") or p.get("kind") or ""
        if can_autofix(kind) and len(auto) < max_auto:
            auto.append(p)
        else:
            required.append(p)
    return {"auto_executed": auto, "approvals_required": required}
