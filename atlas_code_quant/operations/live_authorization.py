"""Explicit human authorization contract for live unlock."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any


def build_live_unlock_request(
    *,
    requested_by: str,
    scope_type: str = "global",
    scope_value: str = "*",
    reason: str = "",
) -> dict[str, Any]:
    return {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "requested_by": str(requested_by or "unknown"),
        "scope_type": str(scope_type or "global"),
        "scope_value": str(scope_value or "*"),
        "reason": str(reason or "manual_unlock_request"),
        "status": "requested",
    }


def approve_live_unlock(
    request_payload: dict[str, Any],
    *,
    approved_by: str,
    approval_reason: str,
) -> dict[str, Any]:
    payload = dict(request_payload or {})
    payload.update(
        {
            "status": "approved",
            "approved_by": str(approved_by or "unknown"),
            "approval_reason": str(approval_reason or "manual_approval"),
            "approved_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        }
    )
    return payload


def deny_live_unlock(
    request_payload: dict[str, Any],
    *,
    denied_by: str,
    deny_reason: str,
) -> dict[str, Any]:
    payload = dict(request_payload or {})
    payload.update(
        {
            "status": "denied",
            "denied_by": str(denied_by or "unknown"),
            "deny_reason": str(deny_reason or "manual_denial"),
            "denied_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        }
    )
    return payload
