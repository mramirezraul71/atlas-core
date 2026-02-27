"""Hash chain for approvals: HMAC-SHA256, verifiable."""
from __future__ import annotations

import hashlib
import hmac
import os
from typing import Any, Dict, List, Optional


def _secret() -> bytes:
    s = (os.getenv("APPROVALS_CHAIN_SECRET") or os.getenv("APPROVALS_CHAIN_SECRET", "CHANGE_ME_LONG_RANDOM") or "").strip()
    return (s or "CHANGE_ME_LONG_RANDOM").encode("utf-8")


def enabled() -> bool:
    return os.getenv("APPROVALS_CHAIN_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def compute_chain_hash(
    prev_hash: str,
    approval_id: str,
    status: str,
    request_hash: str,
    approved_ts: str,
    approved_by: str,
) -> str:
    """
    chain_hash = HMAC_SHA256(secret, prev|id|status|request_hash|ts|by)
    """
    data = f"{prev_hash}|{approval_id}|{status}|{request_hash}|{approved_ts}|{approved_by}"
    return hmac.new(_secret(), data.encode("utf-8"), hashlib.sha256).hexdigest()


def compute_request_hash(payload: Dict[str, Any]) -> str:
    """Hash of payload for integrity (canonical JSON)."""
    import json
    try:
        canonical = json.dumps(payload, sort_keys=True)
    except Exception:
        canonical = str(payload)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:32]


def verify_chain(items: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Recompute chain from first approved event. Return {ok, valid, broken_at_id?, last_hash, count}.
    """
    prev = "0"
    count = 0
    for item in items:
        if item.get("status") != "approved":
            continue
        count += 1
        aid = item.get("id", "")
        status = item.get("status", "approved")
        req_hash = item.get("request_hash") or compute_request_hash(item.get("payload") or {})
        resolved = item.get("resolved_ts") or ""
        resolved_by = item.get("resolved_by") or ""
        expected = compute_chain_hash(prev, aid, status, req_hash, resolved, resolved_by)
        got = item.get("chain_hash")
        if got != expected:
            return {"ok": False, "valid": False, "broken_at_id": aid, "last_hash": prev, "count": count}
        prev = expected
    return {"ok": True, "valid": True, "broken_at_id": None, "last_hash": prev, "count": count}
