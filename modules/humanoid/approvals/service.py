"""Approval service: create, list, approve, reject. Audit + optional memory link."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

from .gate import requires_approval, risk_level, requires_2fa_for_risk


def _notify_telegram_approval_pending(item: Dict[str, Any]) -> None:
    """If Telegram enabled: critical -> inline Aprobar/Rechazar; else HIGH -> plain message."""
    if os.getenv("TELEGRAM_ENABLED", "").strip().lower() not in ("1", "true", "yes"):
        return
    telegram_confirm = os.getenv("OWNER_TELEGRAM_CONFIRM", "").strip().lower() in ("1", "true", "yes")
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        chat_id = (os.getenv("TELEGRAM_CHAT_ID", "") or "").strip()
        if not chat_id:
            return
        bridge = TelegramBridge()
        risk = (item.get("risk") or "high").strip().lower()
        action = item.get("action") or "approval"
        aid = item.get("id") or ""
        if risk == "critical" and telegram_confirm:
            bridge.send_approval_inline(chat_id, aid, action, risk)
        else:
            msg = f"[ATLAS] Approval pendiente {risk.upper()}: id={aid} action={action}. Revisa /ui o POST /approvals/approve"
            bridge.send(chat_id, msg)
    except Exception:
        pass
from .store import create as store_create, list_items, get, approve as store_approve, reject as store_reject


def create(action: str, payload: Dict[str, Any], job_id: Optional[str] = None, run_id: Optional[int] = None, origin_node_id: Optional[str] = None) -> Dict[str, Any]:
    """Enqueue item if it requires approval. Returns {ok, approval_id?, error}."""
    if not requires_approval(action, payload):
        return {"ok": True, "approval_id": None, "error": None}
    try:
        risk = risk_level(action, payload)
        node_id = origin_node_id or (payload.get("origin_node_id") if isinstance(payload, dict) else None) or os.getenv("CLUSTER_NODE_ID")
        item = store_create(action=action, payload=payload, risk=risk, job_id=job_id, run_id=run_id, requires_2fa=requires_2fa_for_risk(risk), origin_node_id=node_id)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event("approvals", "system", "create", True, 0, None, {"id": item["id"], "action": action, "risk": risk}, None)
        except Exception:
            pass
        if risk in ("high", "critical"):
            _notify_telegram_approval_pending(item)
        return {"ok": True, "approval_id": item["id"], "error": None}
    except Exception as e:
        return {"ok": False, "approval_id": None, "error": str(e)}


def list_pending(limit: int = 50) -> List[Dict[str, Any]]:
    return list_items(status="pending", limit=limit)


def list_all(limit: int = 50) -> List[Dict[str, Any]]:
    return list_items(status=None, limit=limit)


def approve(aid: str, resolved_by: str = "api", owner_session_token: Optional[str] = None, signature: Optional[str] = None) -> Dict[str, Any]:
    item = get(aid)
    if item:
        risk = (item.get("risk") or "medium").strip().lower()
        if risk in ("high", "critical"):
            from modules.humanoid.owner.session import validate as session_validate, strict_mode
            if strict_mode() and not session_validate(owner_session_token):
                return {"ok": False, "id": aid, "status": "owner_session_required", "error": "X-Owner-Session required for high/critical"}
    ok = store_approve(aid, resolved_by=resolved_by, signature=signature)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "approve", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    return {"ok": ok, "id": aid, "status": "approved" if ok else "not_found"}


def reject(aid: str, resolved_by: str = "api") -> Dict[str, Any]:
    ok = store_reject(aid, resolved_by=resolved_by)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "reject", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    return {"ok": ok, "id": aid, "status": "rejected" if ok else "not_found"}
