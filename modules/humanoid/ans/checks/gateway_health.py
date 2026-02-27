"""Gateway health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.gateway.store import build_gateway_status
        status = build_gateway_status()
        ok = status.get("error") is None
        return {"ok": ok, "check_id": "gateway_health", "message": status.get("mode", "ok"), "details": status, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "gateway_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": ["retry_gateway_bootstrap"]}
