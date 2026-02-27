"""Governance audit: log mode changes, emergency toggle, blocked actions."""
from __future__ import annotations


def audit_mode_change(from_val: str, to_val: str, reason: str, actor: str, ok: bool) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event(
            actor, "owner" if actor == "api" else "system",
            "governance", "set_mode", ok, 0, None,
            {"from": from_val, "to": to_val, "reason": reason[:200]},
            None,
        )
    except Exception:
        pass


def audit_emergency_change(enable: bool, reason: str, actor: str, ok: bool) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event(
            actor, "owner",
            "governance", "emergency_stop", ok, 0, None,
            {"enable": enable, "reason": reason[:200]},
            None,
        )
    except Exception:
        pass


def audit_blocked(action_kind: str, reason: str, actor: str = "system") -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event(
            actor, "system",
            "governance", "blocked", False, 0, reason,
            {"action_kind": action_kind},
            None,
        )
    except Exception:
        pass
