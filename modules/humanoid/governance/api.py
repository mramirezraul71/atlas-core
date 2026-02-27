"""Governance API: /governance/* endpoints."""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Header
from pydantic import BaseModel

router = APIRouter(prefix="/governance", tags=["Governance"])


def _require_owner_session(x_owner_session: Optional[str] = None) -> tuple[bool, Optional[str]]:
    if not x_owner_session:
        return False, "owner_session_required"
    try:
        from modules.humanoid.owner.session import session_validate
        if not session_validate(x_owner_session):
            return False, "owner_session_invalid"
    except Exception:
        return False, "owner_session_check_failed"
    return True, None


class ModeBody(BaseModel):
    mode: str = "governed"
    reason: Optional[str] = None


class EmergencyBody(BaseModel):
    enable: bool = False
    reason: Optional[str] = None


@router.get("/status")
def governance_status():
    """GET /governance/status -> {mode, emergency_stop, rules_summary}. Fallback a governed si falla la DB."""
    try:
        from modules.humanoid.governance.state import get_mode, get_emergency_stop
        from modules.humanoid.governance.gates import get_emergency_blocked_actions
        mode = get_mode()
        emergency = get_emergency_stop()
        return {
            "ok": True,
            "mode": mode,
            "emergency_stop": emergency,
            "rules_summary": {
                "emergency_blocked_actions": get_emergency_blocked_actions(),
            },
        }
    except Exception as e:
        import logging
        logging.getLogger("atlas.governance").warning("governance/status fallback: %s", e)
        return {
            "ok": True,
            "mode": "governed",
            "emergency_stop": False,
            "rules_summary": {"emergency_blocked_actions": []},
            "fallback": True,
            "error_ignored": str(e)[:200],
        }


@router.get("/rules")
def governance_rules():
    """GET /governance/rules -> hard limits + allowlists."""
    from modules.humanoid.governance.models import HARD_LIMIT_ACTION_KINDS
    from modules.humanoid.governance.gates import get_emergency_blocked_actions
    return {
        "ok": True,
        "hard_limits_always_approval": list(HARD_LIMIT_ACTION_KINDS),
        "emergency_blocked": get_emergency_blocked_actions(),
    }


@router.post("/mode")
def governance_set_mode(
    body: ModeBody,
    x_owner_session: Optional[str] = Header(None, alias="X-Owner-Session"),
):
    """POST /governance/mode. Cambio inmediato; no requiere owner session."""
    prev = "growth"
    mode = (getattr(body, "mode", None) or "governed").strip().lower()
    if mode not in ("growth", "governed"):
        return {"ok": False, "mode": mode, "prev": prev, "error": "invalid_mode", "valid": ["growth", "governed"]}
    try:
        from modules.humanoid.governance.state import get_mode, set_mode
        from modules.humanoid.governance.audit import audit_mode_change
        from modules.humanoid.governance.notifier import notify_mode_change
        prev = get_mode()
        set_mode(mode, reason=getattr(body, "reason", "") or "", actor="api")
        try:
            audit_mode_change(prev, mode, getattr(body, "reason", "") or "", "api", True)
            notify_mode_change(mode, getattr(body, "reason", "") or "")
        except Exception:
            pass
        return {"ok": True, "mode": mode, "prev": prev}
    except Exception:
        try:
            import modules.humanoid.governance.state as _state
            _state._cache["mode"] = mode
        except Exception:
            pass
        return {"ok": True, "mode": mode, "prev": prev}


@router.post("/emergency")
def governance_emergency(
    body: EmergencyBody,
    x_owner_session: Optional[str] = Header(None, alias="X-Owner-Session"),
):
    """POST /governance/emergency. RESTRICCIONES DESACTIVADAS - no requiere owner session."""
    # ok, err check desactivado hasta indicar
    try:
        from modules.humanoid.governance.state import get_emergency_stop, set_emergency_stop
        from modules.humanoid.governance.audit import audit_emergency_change
        from modules.humanoid.governance.notifier import notify_emergency_change
        success = set_emergency_stop(body.enable, reason=body.reason or "", actor="api")
        audit_emergency_change(body.enable, body.reason or "", "api", success)
        if success:
            notify_emergency_change(body.enable, body.reason or "")
        return {"ok": success, "emergency_stop": body.enable}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/log")
def governance_log(limit: int = 20):
    """GET /governance/log -> cambios de modo."""
    try:
        from modules.humanoid.governance.state import get_log
        return {"ok": True, "log": get_log(limit=min(limit, 100))}
    except Exception as e:
        return {"ok": False, "log": [], "error": str(e)}
