"""Gates: paid_api_allowed, external_network_allowed (owner/approval)."""
from __future__ import annotations

import os
from typing import Callable, Optional

def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return (v or "").strip() or default


def _env_bool(name: str, default: bool) -> bool:
    v = _env(name, "true" if default else "false").lower()
    return v in ("1", "true", "yes", "y", "on")


# Optional: inject checker for "owner session" (e.g. from auth middleware)
_owner_session_check: Optional[Callable[[], bool]] = None


def set_owner_session_check(fn: Callable[[], bool]) -> None:
    global _owner_session_check
    _owner_session_check = fn


def has_owner_session() -> bool:
    if _owner_session_check:
        return _owner_session_check()
    return False


def external_network_allowed() -> bool:
    return _env_bool("AI_ALLOW_EXTERNAL_APIS", False)


def paid_api_allowed() -> bool:
    if not external_network_allowed():
        return False
    if _env_bool("AI_REQUIRE_OWNER_SESSION_FOR_PAID", True) and not has_owner_session():
        return False
    return True


def require_approval_for_paid() -> bool:
    return _env_bool("AI_REQUIRE_APPROVAL_FOR_PAID", True)


def approval_risk_for_paid() -> str:
    return _env("AI_APPROVAL_RISK_FOR_PAID", "high").strip().lower() or "high"
