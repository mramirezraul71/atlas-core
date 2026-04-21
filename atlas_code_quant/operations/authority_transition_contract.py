"""Authority transition contract for staged paper->live handoff."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any


def build_authority_transition_assessment(
    *,
    initiator: str,
    authority_level: str,
    recommendation: str,
    transition_reason: str,
    rollback_ready: bool,
    emergency_stop_ready: bool,
    required_human_ack: bool = True,
    additional_checks: list[str] | None = None,
) -> dict[str, Any]:
    initiator_norm = str(initiator or "system").lower()
    authority_norm = str(authority_level or "paper").lower()
    recommendation_norm = str(recommendation or "stay_paper").lower()
    checks = list(additional_checks or [])
    return {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "initiator": initiator_norm,
        "authority_level": authority_norm,
        "recommended_transition": recommendation_norm,
        "required_human_ack": bool(required_human_ack),
        "transition_reason": str(transition_reason or "readiness_assessment"),
        "rollback_ready": bool(rollback_ready),
        "emergency_stop_ready": bool(emergency_stop_ready),
        "additional_checks": checks,
    }
