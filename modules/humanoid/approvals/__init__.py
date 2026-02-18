"""Approval queue: policy-gated actions confirmable via UI/Telegram/Voice."""
from __future__ import annotations

import time
from typing import Any, Dict, Optional

from .gate import requires_approval, risk_level, requires_2fa_for_risk
from .service import create, list_pending, list_all, approve, reject
from .store import get, verify_chain, list_items

def create_approval(
    *,
    action: str,
    risk: str = "high",
    description: str = "",
    payload: Optional[Dict[str, Any]] = None,
    job_id: Optional[str] = None,
    run_id: Optional[int] = None,
    origin_node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Wrapper estable para el Dispatcher.
    Devuelve: {ok, id, error}
    """
    pl = dict(payload or {})
    if description and not pl.get("description"):
        pl["description"] = description
    if risk and not pl.get("risk"):
        pl["risk"] = risk
    r = create(action=action, payload=pl, job_id=job_id, run_id=run_id, origin_node_id=origin_node_id)
    return {"ok": bool(r.get("ok")), "id": r.get("approval_id"), "error": r.get("error")}


def wait_for_resolution(approval_id: str, timeout_seconds: int = 300, poll_seconds: float = 1.5) -> Dict[str, Any]:
    """
    Espera hasta que una aprobación pase de pending -> approved/rejected o expire/timeout.
    Devuelve: {ok, id, status, error}
    """
    end = time.time() + max(1, int(timeout_seconds or 300))
    while time.time() < end:
        item = get(approval_id)
        if not item:
            return {"ok": False, "id": approval_id, "status": "not_found", "error": "not_found"}
        st = (item.get("status") or "").strip().lower()
        if st and st != "pending":
            return {"ok": True, "id": approval_id, "status": st, "error": None}
        if item.get("expired"):
            return {"ok": False, "id": approval_id, "status": "expired", "error": "expired"}
        time.sleep(max(0.2, float(poll_seconds)))
    return {"ok": False, "id": approval_id, "status": "timeout", "error": "timeout"}


__all__ = [
    "requires_approval", "risk_level", "requires_2fa_for_risk",
    "create", "list_pending", "list_all", "approve", "reject",
    "get", "verify_chain", "list_items",
    "create_approval", "wait_for_resolution",
]
