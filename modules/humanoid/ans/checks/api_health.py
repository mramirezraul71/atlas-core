"""API health check."""
from __future__ import annotations

import os


def run() -> dict:
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        port = int(os.getenv("ACTIVE_PORT", "8791"))
        h = run_health_verbose(base_url=None, active_port=port)
        ok = h.get("ok", False)
        score = h.get("score", 0)
        return {
            "ok": ok and score >= 50,
            "check_id": "api_health",
            "message": f"score={score} ok={ok}",
            "details": {"score": score, "uptime_sec": h.get("uptime_sec"), "ms": h.get("ms")},
            "severity": "high" if not ok else "low",
            "suggested_heals": ["clear_stale_locks", "restart_scheduler"] if not ok else [],
        }
    except Exception as e:
        return {"ok": False, "check_id": "api_health", "message": str(e), "details": {"error": str(e)}, "severity": "high", "suggested_heals": ["clear_stale_locks", "restart_scheduler"]}
