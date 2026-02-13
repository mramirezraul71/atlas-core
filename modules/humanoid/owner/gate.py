"""Owner gate: enforce session for risky actions. Zero-trust."""
from __future__ import annotations

import os
from typing import Optional

from .session import validate as session_validate
from .emergency import is_action_blocked


def owner_enabled() -> bool:
    return os.getenv("OWNER_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def owner_id() -> str:
    return (os.getenv("OWNER_ID", "owner") or "owner").strip()


def require_session_for_risk() -> str:
    """low|medium|high|critical - actions at or above need session."""
    return (os.getenv("OWNER_REQUIRE_SESSION_FOR_RISK", "medium") or "medium").strip().lower()


def risk_ordinal(r: str) -> int:
    o = {"low": 0, "medium": 1, "high": 2, "critical": 3}
    return o.get((r or "").strip().lower(), 0)


def requires_owner_session(risk: str) -> bool:
    """True if risk level requires owner session."""
    threshold = require_session_for_risk()
    return risk_ordinal(risk) >= risk_ordinal(threshold)


def check_owner_gate(
    risk: str,
    session_token: Optional[str],
    action: Optional[str] = None,
) -> tuple[bool, Optional[str]]:
    """
    Returns (allow, error_message).
    RESTRICCIONES DESACTIVADAS HASTA INDICAR - siempre permite.
    """
    return True, None  # Sin restricciones
