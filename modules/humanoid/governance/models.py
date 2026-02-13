"""Governance models: GovernanceMode, ActionKind, Decision."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class GovernanceMode(str, Enum):
    GROWTH = "growth"
    GOVERNED = "governed"


class ActionKind(str, Enum):
    CODE_CHANGE = "code_change"
    DEPS_CHANGE = "deps_change"
    REFACTOR = "refactor"
    UPDATE_APPLY = "update_apply"
    DEPLOY_APPLY = "deploy_apply"
    REMOTE_EXECUTE = "remote_execute"
    SCREEN_ACT_DESTRUCTIVE = "screen_act_destructive"
    SHELL_EXEC = "shell_exec"
    CONFIG_CHANGE_CRITICAL = "config_change_critical"
    CONFIG_CHANGE_SAFE = "config_change_safe"
    ANS_HEAL = "ans_heal"
    GA_AUTORUN = "ga_autorun"
    SELFPROG_APPLY = "selfprog_apply"
    CURSOR_TOOL_EXEC = "cursor_tool_exec"


@dataclass
class Decision:
    allow: bool
    blocked_by_emergency: bool
    needs_approval: bool
    reason: str
    approval_id: Optional[str] = None


HARD_LIMIT_ACTION_KINDS = frozenset({
    "secrets_keys_change",
    "network_tunnel_firewall",
    "deletion_audit_snapshots_policy_rollback",
    "destructive_db",
})
