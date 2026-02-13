"""LLM health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.deploy.healthcheck import _check_llm_reachable
        r = _check_llm_reachable()
        ok = r.get("ok", False)
        return {
            "ok": ok,
            "check_id": "llm_health",
            "message": r.get("message", "ok" if ok else "reachable"),
            "details": r,
            "severity": "med" if not ok else "low",
            "suggested_heals": ["fallback_models"] if not ok else [],
        }
    except Exception as e:
        return {"ok": False, "check_id": "llm_health", "message": str(e), "details": {"error": str(e)}, "severity": "med", "suggested_heals": ["fallback_models"]}
