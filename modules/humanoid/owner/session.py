"""Owner session: TTL token for high/critical approvals. X-Owner-Session header."""
from __future__ import annotations

import os
import secrets
import time
from typing import Any, Dict, Optional

_SESSIONS: Dict[str, Dict[str, Any]] = {}
_TIMEOUT = 900


def _timeout_sec() -> int:
    try:
        return int(os.getenv("OWNER_SESSION_TIMEOUT", "900") or 900)
    except (TypeError, ValueError):
        return _TIMEOUT


def owner_id() -> str:
    return (os.getenv("OWNER_ID", "owner") or "owner").strip()


def strict_mode() -> bool:
    return os.getenv("OWNER_STRICT_MODE", "true").strip().lower() in ("1", "true", "yes")


def start(actor: str, method: str = "ui") -> Dict[str, Any]:
    """Generate session token with TTL. method: ui | telegram | voice."""
    if strict_mode() and actor != owner_id():
        return {"ok": False, "session_token": None, "error": "actor not owner"}
    method = (method or "ui").strip().lower()
    if method not in ("ui", "telegram", "voice"):
        method = "ui"
    token = secrets.token_urlsafe(32)
    ttl = _timeout_sec()
    _SESSIONS[token] = {"actor": actor, "method": method, "created_at": time.time(), "ttl": ttl}
    return {"ok": True, "session_token": token, "ttl_seconds": ttl, "method": method}


def validate(token: Optional[str]) -> bool:
    """Return True if token is valid and not expired."""
    if not token or token not in _SESSIONS:
        return False
    s = _SESSIONS[token]
    if time.time() - s["created_at"] > s["ttl"]:
        del _SESSIONS[token]
        return False
    return True


def list_active() -> list:
    """List active sessions (for UI). Expire stale first."""
    ttl = _timeout_sec()
    expired = [t for t, s in _SESSIONS.items() if time.time() - s["created_at"] > (s.get("ttl") or ttl)]
    for t in expired:
        _SESSIONS.pop(t, None)
    return [{"method": s.get("method"), "created_at": s.get("created_at")} for s in _SESSIONS.values()]
