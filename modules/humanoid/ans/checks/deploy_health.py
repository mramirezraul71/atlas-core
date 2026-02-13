"""Deploy health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.deploy.bluegreen import get_status
        s = get_status()
        ok = s.get("ok", True)
        return {"ok": ok, "check_id": "deploy_health", "message": s.get("mode", "ok"), "details": s, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "deploy_health", "message": str(e), "details": {"error": str(e)}, "severity": "low"}
