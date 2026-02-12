"""Gateway events to audit. Privacy: redact tokens in payloads."""
from __future__ import annotations

from typing import Any, Dict, Optional


def _redact(payload: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not payload:
        return payload
    out = dict(payload)
    for key in ("token", "CLOUDFLARE_TOKEN", "secret", "password"):
        if key in out and out[key]:
            out[key] = "[REDACTED]"
    return out


def log_gateway_event(action: str, ok: bool, payload: Optional[Dict[str, Any]] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("gateway", "system", "gateway", action, ok, ms, error, _redact(payload), None)
    except Exception:
        pass
