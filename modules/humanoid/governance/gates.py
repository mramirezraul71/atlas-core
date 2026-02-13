"""Decide allow/need_approval/blocked_by_emergency per action."""
from __future__ import annotations

import os
from typing import Optional, Tuple

from .models import ActionKind, Decision
from .state import get_mode, get_emergency_stop

_EMERGENCY_BLOCKED = frozenset({
    "deploy_apply", "code_change", "deps_change", "remote_execute", "shell_exec",
    "update_apply", "refactor", "ans_heal", "ga_autorun", "selfprog_apply", "cursor_tool_exec",
})


def get_emergency_blocked_actions() -> list:
    return list(_EMERGENCY_BLOCKED)


def _env_bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _action_kind_str(kind: str) -> str:
    return (kind or "").strip().lower().replace(" ", "_")


def is_blocked_by_emergency(action_kind: str) -> bool:
    """True si EMERGENCY_STOP y la acción está bloqueada."""
    if not get_emergency_stop():
        return False
    k = _action_kind_str(action_kind)
    if k in _EMERGENCY_BLOCKED:
        return True
    env_map = {
        "deploy_apply": "EMERGENCY_BLOCK_DEPLOY",
        "code_change": "EMERGENCY_BLOCK_CODE_CHANGES",
        "deps_change": "EMERGENCY_BLOCK_DEPS",
        "remote_execute": "EMERGENCY_BLOCK_REMOTE_EXEC",
        "shell_exec": "EMERGENCY_BLOCK_SHELL",
    }
    return _env_bool(env_map.get(k, "EMERGENCY_BLOCK_DEPLOY"), True)


def needs_approval(action_kind: str) -> bool:
    """True si mode=governed y la acción requiere approval."""
    if get_mode() != "governed":
        return False
    k = _action_kind_str(action_kind)
    env_map = {
        "code_change": "GOVERNED_REQUIRE_APPROVAL_FOR_CODE",
        "deps_change": "GOVERNED_REQUIRE_APPROVAL_FOR_DEPS",
        "update_apply": "GOVERNED_REQUIRE_APPROVAL_FOR_UPDATE",
        "refactor": "GOVERNED_REQUIRE_APPROVAL_FOR_REFACTOR",
        "remote_execute": "GOVERNED_REQUIRE_APPROVAL_FOR_REMOTE_EXEC",
        "deploy_apply": "GOVERNED_REQUIRE_APPROVAL_FOR_UPDATE",
        "ans_heal": "GOVERNED_REQUIRE_APPROVAL_FOR_CODE",
        "ga_autorun": "GOVERNED_REQUIRE_APPROVAL_FOR_CODE",
        "selfprog_apply": "GOVERNED_REQUIRE_APPROVAL_FOR_CODE",
        "cursor_tool_exec": "GOVERNED_REQUIRE_APPROVAL_FOR_CODE",
    }
    return _env_bool(env_map.get(k, "GOVERNED_REQUIRE_APPROVAL_FOR_CODE"), True)


def decide(action_kind: str, context: Optional[dict] = None) -> Decision:
    """
    Decide allow/need_approval/blocked_by_emergency.
    1) EMERGENCY_STOP + blocked => error emergency_stop_block
    2) mode=governed + needs_approval => needs_approval=True, allow=False (crear approval)
    3) mode=growth => allow=True salvo hard limits (estos siempre approval)
    """
    k = _action_kind_str(action_kind)
    blocked = is_blocked_by_emergency(k)
    if blocked:
        return Decision(
            allow=False,
            blocked_by_emergency=True,
            needs_approval=False,
            reason="emergency_stop_block",
        )
    if needs_approval(k):
        return Decision(
            allow=False,
            blocked_by_emergency=False,
            needs_approval=True,
            reason="governed_requires_approval",
            approval_id=None,
        )
    return Decision(
        allow=True,
        blocked_by_emergency=False,
        needs_approval=False,
        reason="allowed",
    )
