"""Approval TTL: expiration handling."""
from __future__ import annotations

import os
from datetime import datetime, timedelta, timezone
from typing import Optional


def _default_ttl_seconds() -> int:
    sec = os.getenv("OWNER_APPROVAL_DEFAULT_TTL_SECONDS")
    if sec:
        try:
            return int(sec)
        except (TypeError, ValueError):
            pass
    try:
        return int(os.getenv("APPROVAL_EXPIRE_MINUTES", "15") or 15) * 60
    except (TypeError, ValueError):
        return 900


def expires_at_seconds(seconds: Optional[int] = None) -> str:
    """ISO string for now + seconds."""
    s = seconds if seconds is not None else _default_ttl_seconds()
    return (datetime.now(timezone.utc) + timedelta(seconds=s)).isoformat()


def is_expired(expires_at: Optional[str]) -> bool:
    if not expires_at:
        return False
    try:
        s = expires_at.replace("Z", "+00:00")
        exp = datetime.fromisoformat(s)
        if exp.tzinfo is None:
            exp = exp.replace(tzinfo=timezone.utc)
        return datetime.now(timezone.utc) >= exp
    except Exception:
        return False
