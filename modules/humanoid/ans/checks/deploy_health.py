"""Deploy health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.deploy.switcher import get_deploy_state
        s = get_deploy_state()
        mode = s.get("mode", "single")
        ok = True
        return {"ok": ok, "check_id": "deploy_health", "message": mode, "details": s, "severity": "low", "suggested_heals": []}
    except Exception as e:
        return {"ok": False, "check_id": "deploy_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": ["restart_scheduler"]}
