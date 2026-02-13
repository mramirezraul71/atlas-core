"""Router health: AI routing ok."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.ai.status import get_ai_status
        s = get_ai_status()
        ok = s.get("ollama_available", False) or len(s.get("route_to_model", {})) > 0
        return {"ok": ok, "check_id": "router_health", "message": "ok" if ok else "no models", "details": {"ollama_available": s.get("ollama_available")}, "severity": "med" if not ok else "low"}
    except Exception as e:
        return {"ok": False, "check_id": "router_health", "message": str(e), "details": {"error": str(e)}, "severity": "low"}
