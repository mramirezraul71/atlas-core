"""Policy engine: can(actor, module, action, target) -> PolicyDecision. Deny-by-default in strict."""
from __future__ import annotations

import os
import shlex
from pathlib import Path
from typing import Any, Dict, List, Optional

from .defaults import get_allowed_command_prefixes, get_default_policies
from .models import ActorContext, PolicyDecision, PolicyRule


def _strict_mode() -> bool:
    v = os.getenv("POLICY_MODE", "strict").strip().lower()
    return v == "strict"


def _allowed_paths() -> List[Path]:
    v = os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")
    return [Path(x.strip()) for x in v.split(",") if x.strip()]


class PolicyEngine:
    """Evaluates can(actor, module, action, target). Strict mode: no explicit allow => deny."""

    def __init__(self, rules: Optional[List[PolicyRule]] = None) -> None:
        self._rules: List[PolicyRule] = list(rules or get_default_policies())
        self._allowed_prefixes = get_allowed_command_prefixes()
        self._allowed_paths = _allowed_paths()

    def can(self, actor: ActorContext, module: str, action: str, target: Any = None) -> PolicyDecision:
        """Return PolicyDecision(allow, reason). In strict mode, default is deny."""
        ctx: Dict[str, Any] = {
            "actor": actor.actor,
            "role": actor.role,
            "module": module,
            "action": action,
            "target": target,
        }
        # Resolve derived context for hands
        if module == "hands" and action == "exec_command" and isinstance(target, str):
            prefix = (shlex.split(target) or [target])[0].strip().lower() if target else ""
            ctx["prefix_allowed"] = prefix in [p.lower() for p in self._allowed_prefixes]
        if module == "hands" and action == "read_file" and target:
            p = Path(target).resolve()
            try:
                ctx["path_allowed"] = any(p == a.resolve() or (hasattr(p, "is_relative_to") and p.is_relative_to(a.resolve())) for a in self._allowed_paths)
            except (ValueError, OSError):
                ctx["path_allowed"] = False
        if module == "update" and action == "apply":
            ctx["action"] = "apply"
        if module == "hands" and action == "kill_process":
            ctx["action"] = "kill_process"

        allowed = False
        deny_reason = ""
        for rule in self._rules:
            if rule.scope != module:
                continue
            if not self._matches(rule.condition, ctx):
                continue
            if rule.action == "deny":
                return PolicyDecision(allow=False, reason=(rule.meta or {}).get("description", "denied"), details={"rule_id": rule.id})
            if rule.action == "allow":
                allowed = True

        if _strict_mode() and not allowed:
            return PolicyDecision(allow=False, reason="strict: no matching allow rule", details={"module": module, "action": action})
        if allowed:
            return PolicyDecision(allow=True, reason="ok", details=ctx)
        return PolicyDecision(allow=False, reason=deny_reason or "denied", details=ctx)

    def _matches(self, condition: Dict[str, Any], context: Dict[str, Any]) -> bool:
        for key, expected in condition.items():
            if context.get(key) != expected:
                return False
        return True


_policy_engine: Optional[PolicyEngine] = None


def get_policy_engine() -> PolicyEngine:
    """Singleton PolicyEngine for app-wide use."""
    global _policy_engine
    if _policy_engine is None:
        _policy_engine = PolicyEngine()
    return _policy_engine
