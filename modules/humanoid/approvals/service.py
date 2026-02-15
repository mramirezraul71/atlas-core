"""Approval service: create, list, approve, reject. Audit + optional memory link."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

from .gate import requires_approval, risk_level, requires_2fa_for_risk


def _notify_telegram_approval_pending(item: Dict[str, Any]) -> None:
    """Push a Telegram for ANY high/critical approval (owner away from dashboard)."""
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        from modules.humanoid.comms.ops_bus import _telegram_chat_id  # type: ignore

        # Enabled by default if token+chat_id exist; can be disabled explicitly.
        enabled = (os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "true") or "").strip().lower()
        if enabled not in ("1", "true", "yes"):
            return
        chat_id = (_telegram_chat_id() or "").strip()
        if not chat_id:
            return
        bridge = TelegramBridge()
        risk = (item.get("risk") or "high").strip().lower()
        action = item.get("action") or "approval"
        aid = item.get("id") or ""
        expires = item.get("expires_at") or ""
        node = item.get("origin_node_id") or ""
        req2fa = bool(item.get("requires_2fa")) if item.get("requires_2fa") is not None else False

        # Siempre enviar con botones inline (más rápido desde móvil).
        bridge.send_approval_inline(chat_id, aid, action, risk=risk)
        # Contexto adicional en lenguaje humano (sin símbolos/IDs largos)
        parts = []
        if expires:
            parts.append(f"Expira: {str(expires)[:19].replace('T',' ')}")
        if node:
            parts.append("Origen: nodo del cluster")
        if req2fa:
            parts.append("Seguridad: requerirá confirmación adicional")
        if parts:
            bridge.send(chat_id, "<b>Detalles</b>\n" + "\n".join("- " + p for p in parts))
    except Exception:
        pass


def _notify_telegram_approval_resolved(item: Optional[Dict[str, Any]], aid: str, status: str, resolved_by: str) -> None:
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        from modules.humanoid.comms.ops_bus import _telegram_chat_id  # type: ignore

        enabled = (os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "true") or "").strip().lower()
        if enabled not in ("1", "true", "yes"):
            return
        chat_id = (_telegram_chat_id() or "").strip()
        if not chat_id:
            return
        st = (status or "").strip().lower()
        if st == "approved":
            msg = "Aprobación ejecutada: APROBADA."
        elif st == "rejected":
            msg = "Aprobación ejecutada: RECHAZADA."
        else:
            msg = "Aprobación: estado actualizado."
        TelegramBridge().send(chat_id, f"<b>ATLAS</b>\n{msg}\nID: <code>{aid}</code>")
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
        # Autonomía: TODA aprobación debe llegar a Telegram (owner puede estar fuera del dashboard).
        _notify_telegram_approval_pending(item)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            lvl = "high" if risk in ("high", "critical") else "med"
            ops_emit(
                "approval",
                "Aprobación pendiente. Te la envié por Telegram para que la resuelvas.",
                level=lvl,
                data={"id": item.get("id"), "action": action, "risk": risk},
            )
        except Exception:
            pass
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
    from .ttl import is_expired
    item = get(aid)
    if item:
        # Idempotencia: si ya está resuelta, no fallar (evita re-aprobar en Telegram)
        st0 = (item.get("status") or "").strip().lower()
        if st0 == "approved":
            return {"ok": True, "id": aid, "status": "already_approved", "error": None}
        if st0 == "rejected":
            return {"ok": False, "id": aid, "status": "already_rejected", "error": "Ya fue rechazada."}
        risk = (item.get("risk") or "medium").strip().lower()
        allow, err = check_owner_gate(risk, owner_session_token, action=None)
        if not allow:
            return {"ok": False, "id": aid, "status": "owner_session_required", "error": err or "X-Owner-Session required"}
        if is_expired(item.get("expires_at")):
            return {"ok": False, "id": aid, "status": "expired", "error": "Approval expirado. Rechaza y regenera el plan."}
        if confirm_token and not consume_nonce(confirm_token):
            return {"ok": False, "id": aid, "status": "replay_or_invalid", "error": "Invalid or reused confirm_token (nonce)"}
    ok = store_approve(aid, resolved_by=resolved_by, signature=signature)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "approve", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    try:
        from modules.humanoid.metalearn.collector import record_approval_resolved
        record_approval_resolved(aid, item.get("action"), item.get("risk"), "approved", item)
    except Exception:
        pass
    # Si existía item pero no aprobó, distinguir expiración vs no encontrado.
    if not ok and item:
        try:
            from .ttl import is_expired
            if is_expired(item.get("expires_at")):
                return {"ok": False, "id": aid, "status": "expired", "error": "Approval expirado. Rechaza y regenera el plan."}
        except Exception:
            pass
        return {"ok": False, "id": aid, "status": "failed", "error": "No se pudo aprobar (estado inválido o condición de seguridad)."}
    if ok:
        _notify_telegram_approval_resolved(item, aid, "approved", resolved_by=resolved_by)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("approval", f"Aprobación APROBADA: id={aid} action={(item or {}).get('action')}", level="info", data={"id": aid, "status": "approved"})
        except Exception:
            pass
    return {"ok": ok, "id": aid, "status": "approved" if ok else "not_found", "error": None if ok else "not_found"}


def reject(aid: str, resolved_by: str = "api") -> Dict[str, Any]:
    item = get(aid)
    if item:
        # Idempotencia: si ya está resuelta, no fallar (evita re-rechazar en Telegram)
        st0 = (item.get("status") or "").strip().lower()
        if st0 == "rejected":
            return {"ok": True, "id": aid, "status": "already_rejected"}
        if st0 == "approved":
            return {"ok": False, "id": aid, "status": "already_approved"}
    ok = store_reject(aid, resolved_by=resolved_by)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "reject", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    try:
        from modules.humanoid.metalearn.collector import record_approval_resolved
        record_approval_resolved(aid, (item or {}).get("action"), (item or {}).get("risk"), "rejected", item or {})
    except Exception:
        pass
    if ok:
        _notify_telegram_approval_resolved(item, aid, "rejected", resolved_by=resolved_by)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("approval", f"Aprobación RECHAZADA: id={aid} action={(item or {}).get('action')}", level="info", data={"id": aid, "status": "rejected"})
        except Exception:
            pass
    return {"ok": ok, "id": aid, "status": "rejected" if ok else "not_found"}
