"""Policy gate: determine if an action requires approval (not safe autofix / read-only)."""
from __future__ import annotations

from typing import Any, Dict, Optional

from .risk import risk_level as _risk_level, requires_2fa_for_risk as _requires_2fa

# Actions that do NOT require approval (safe or read-only)
SAFE_OR_READ_ACTIONS = frozenset({
    "read", "status", "recall", "export", "list", "check", "plan_only",
    "memory_read", "memory_export", "ci_autofix",
})


def requires_approval(action: str, payload: Optional[Dict[str, Any]] = None) -> bool:
    """True if this action must go to the approval queue (not safe/read)."""
    action_lower = (action or "").strip().lower()
    if action_lower in SAFE_OR_READ_ACTIONS:
        return False
    # Acciones sensibles aunque no contengan "apply/execute" en el nombre
    if action_lower in ("shell_exec", "remote_execute", "screen_act_destructive"):
        # Si payload trae riesgo alto/crítico, siempre requiere aprobación
        try:
            r = ((payload or {}).get("risk") or "").strip().lower()
            if r in ("high", "critical"):
                return True
        except Exception:
            return True
        # Por defecto, requerir aprobación (mejor seguro que roto)
        return True
    if action_lower in ("execute", "apply", "run", "update_apply", "promote", "run_now"):
        return True
    if "apply" in action_lower or "execute" in action_lower or "delete" in action_lower:
        return True
    return False


def risk_level(action: str, payload: Optional[Dict[str, Any]] = None) -> str:
    return _risk_level(action, payload)


def requires_2fa_for_risk(risk: str) -> bool:
    return _requires_2fa(risk)
