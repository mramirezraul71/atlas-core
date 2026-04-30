"""Approval and safety policy for atlas_agent tool actions."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable

try:
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig


@dataclass
class PolicyDecision:
    allowed: bool
    requires_approval: bool
    reason: str


class ApprovalPolicyEngine:
    """Decides whether a tool action is allowed, blocked, or requires approval."""

    SAFE_ALLOW = {"list_files", "read_file", "search_text"}
    AGGRESSIVE_AUTO_ALLOW = {"write_file"}
    SHELL_SAFE_PREFIXES = (
        "rg ",
        "python -m pytest",
        "python -m compileall",
        "Get-ChildItem",
        "Get-Content",
        "Select-String",
        "git status",
        "git diff",
    )

    def __init__(
        self,
        config: AgentConfig,
        approval_overrides: Dict[str, bool] | None = None,
        approved_tools: Iterable[str] | None = None,
    ):
        self.config = config
        self.approval_overrides = {
            (k or "").strip().lower(): bool(v)
            for k, v in (approval_overrides or {}).items()
        }
        self.approved_tools = {(x or "").strip().lower() for x in (approved_tools or [])}

    def decide(self, tool: str, args: Dict[str, Any] | None = None) -> PolicyDecision:
        args = args or {}
        t = (tool or "").strip().lower()
        if t in self.approval_overrides:
            override = self.approval_overrides[t]
            return PolicyDecision(
                allowed=override,
                requires_approval=False,
                reason=f"policy override: {override}",
            )
        if t in self.approved_tools:
            return PolicyDecision(True, False, "approved by request")

        if t in self.SAFE_ALLOW:
            return PolicyDecision(True, False, "read-only tool")

        if t == "write_file":
            if self.config.mode == "aggressive":
                return PolicyDecision(True, False, "aggressive mode auto-approve write")
            return PolicyDecision(False, True, "write requires explicit approval in safe mode")

        if t == "run_shell":
            if not self.config.allow_shell:
                return PolicyDecision(False, False, "shell disabled by config")
            cmd = str(args.get("command") or "").strip()
            cmd_l = cmd.lower()
            if self.config.mode == "aggressive":
                if any(cmd_l.startswith(prefix.lower()) for prefix in self.SHELL_SAFE_PREFIXES):
                    return PolicyDecision(True, False, "aggressive mode safe-shell allow")
                return PolicyDecision(False, True, "shell requires approval for non-safe command")
            return PolicyDecision(False, True, "shell requires explicit approval in safe mode")

        return PolicyDecision(False, False, f"tool not authorized: {tool}")
