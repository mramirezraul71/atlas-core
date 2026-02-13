"""Approval service: create, list, approve, reject. Audit + optional memory link."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

from .gate import requires_approval, risk_level, requires_2fa_for_risk


def _notify_telegram_approval_pending(item: Dict[str, Any]) -> None:
    """If Telegram enabled: critical -> inline Aprobar/Rechazar; else HIGH -> plain message."""
    enabled = os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "")
    if enabled.strip().lower() not in ("1", "true", "yes"):
        return
    telegram_confirm = (os.getenv("TELEGRAM_REQUIRE_CONFIRM") or os.getenv("OWNER_TELEGRAM_CONFIRM", "")).strip().lower() in ("1", "true", "yes")
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        raw = (os.getenv("TELEGRAM_ALLOWED_CHAT_IDS") or os.getenv("TELEGRAM_CHAT_ID", "") or "").strip()
        chat_ids = [x.strip() for x in raw.replace(",", " ").split() if x.strip()]
        chat_id = chat_ids[0] if chat_ids else ""
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


def list_pending(limit: int = 50, risk: Optional[str] = None) -> List[Dict[str, Any]]:
    return list_items(status="pending", risk=risk, limit=limit)


def list_all(limit: int = 50, status: Optional[str] = None, risk: Optional[str] = None) -> List[Dict[str, Any]]:
    return list_items(status=status, risk=risk, limit=limit)


def approve(
    aid: str,
    resolved_by: str = "api",
    owner_session_token: Optional[str] = None,
    signature: Optional[str] = None,
    approved_via: str = "api",
    confirm_token: Optional[str] = None,
) -> Dict[str, Any]:
    from modules.humanoid.owner.gate import check_owner_gate
    from .replay import consume_nonce
    item = get(aid)
    if item:
        risk = (item.get("risk") or "medium").strip().lower()
        allow, err = check_owner_gate(risk, owner_session_token, action=None)
        if not allow:
            return {"ok": False, "id": aid, "status": "owner_session_required", "error": err or "X-Owner-Session required"}
        if confirm_token and not consume_nonce(confirm_token):
            return {"ok": False, "id": aid, "status": "replay_or_invalid", "error": "Invalid or reused confirm_token (nonce)"}
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
