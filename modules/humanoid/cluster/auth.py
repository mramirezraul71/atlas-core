"""Cluster auth: HMAC sign/verify + nonce replay protection."""
from __future__ import annotations

import hashlib
import hmac
import os
import time
from typing import Optional

from . import db as cluster_db


def _secret() -> bytes:
    s = os.getenv("CLUSTER_SHARED_SECRET", "CHANGE_ME_LONG_RANDOM")
    return s.encode("utf-8") if isinstance(s, str) else s


def _sign_payload(node_id: str, ts: str, nonce: str, method: str, path: str, body_hash: str) -> str:
    payload = f"{node_id}|{ts}|{nonce}|{method}|{path}|{body_hash}"
    return hmac.new(_secret(), payload.encode("utf-8"), hashlib.sha256).hexdigest()


def sign_request(node_id: str, method: str, path: str, body: Optional[bytes] = None) -> tuple[str, str, str]:
    """Return (ts, nonce, signature) for headers."""
    ts = str(int(time.time()))
    import uuid
    nonce = str(uuid.uuid4())
    body_hash = hashlib.sha256(body or b"").hexdigest()
    sig = _sign_payload(node_id, ts, nonce, method.upper(), path, body_hash)
    return ts, nonce, sig


def verify_request(
    node_id: str,
    ts: str,
    nonce: str,
    method: str,
    path: str,
    body: Optional[bytes],
    signature: str,
    ttl_sec: Optional[int] = None,
) -> bool:
    """Verify HMAC and replay (nonce). Reject if nonce seen or ts too old."""
    if ttl_sec is None:
        ttl_sec = int(os.getenv("CLUSTER_TOKEN_TTL_SECONDS", "3600"))
    try:
        ts_int = int(ts)
        if abs(time.time() - ts_int) > ttl_sec:
            return False
    except (TypeError, ValueError):
        return False
    if cluster_db.nonce_seen(nonce, float(ttl_sec * 2)):
        return False
    body_hash = hashlib.sha256(body or b"").hexdigest()
    expected = _sign_payload(node_id, ts, nonce, method.upper(), path, body_hash)
    return hmac.compare_digest(expected, signature)
