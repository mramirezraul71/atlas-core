"""Owner session: TTL token for approvals. X-Owner-Session header. create/validate/revoke."""
from __future__ import annotations

import os
import secrets
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

_SESSIONS: Dict[str, Dict[str, Any]] = {}


def _timeout_sec() -> int:
    v = os.getenv("OWNER_SESSION_TTL_SECONDS") or os.getenv("OWNER_SESSION_TIMEOUT", "900")
    try:
        return int(v or 900)
    except (TypeError, ValueError):
        return 900


def owner_id() -> str:
    return (os.getenv("OWNER_ID", "owner") or "owner").strip()


def strict_mode() -> bool:
    return os.getenv("OWNER_STRICT_MODE", "false").strip().lower() in ("1", "true", "yes")


def _expires_at_iso(created_at: float, ttl: int) -> str:
    return datetime.fromtimestamp(created_at + ttl, tz=timezone.utc).isoformat()


def start(actor: str, method: str = "ui") -> Dict[str, Any]:
    """Generate session token with TTL. method: ui | telegram | voice | api | face | windows."""
    method = (method or "ui").strip().lower()
    if method not in ("ui", "telegram", "voice", "api"):
        method = "ui"
    token = secrets.token_urlsafe(32)
    ttl = _timeout_sec()
    now = time.time()
    _SESSIONS[token] = {
        "actor": actor,
        "method": method,
        "created_at": now,
        "ttl": ttl,
        "expires_at": _expires_at_iso(now, ttl),
    }
    return {
        "ok": True,
        "session_token": token,
        "ttl_seconds": ttl,
        "method": method,
        "expires_at": _expires_at_iso(now, ttl),
    }


def validate(token: Optional[str]) -> bool:
    """Return True if token is valid and not expired."""
    if not token or token not in _SESSIONS:
        return False
    s = _SESSIONS[token]
    if time.time() - s["created_at"] > s["ttl"]:
        _SESSIONS.pop(token, None)
        return False
    return True


def end(session_token: Optional[str]) -> bool:
    """Revoke session. Returns True if revoked."""
    if not session_token:
        return False
    if session_token in _SESSIONS:
        del _SESSIONS[session_token]
        return True
    return False


def list_active() -> list:
    """List active sessions (redacted tokens). Expire stale first."""
    ttl = _timeout_sec()
    now = time.time()
    expired = [t for t, s in _SESSIONS.items() if now - s["created_at"] > (s.get("ttl") or ttl)]
    for t in expired:
        _SESSIONS.pop(t, None)
    return [
        {"method": s.get("method"), "created_at": s.get("created_at"), "expires_at": s.get("expires_at")}
        for s in _SESSIONS.values()
    ]
