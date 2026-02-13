"""Deps health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.deps_checker import check_all
        r = check_all()
        missing = r.get("missing_deps", [])
        ok = len(missing) == 0 or all(m in ("ollama running",) for m in missing)
        return {
            "ok": ok,
            "check_id": "deps_health",
            "message": f"missing={missing}" if missing else "ok",
            "details": {"missing_deps": missing},
            "severity": "low" if ok else "med",
            "suggested_heals": [],
        }
    except Exception as e:
        return {"ok": False, "check_id": "deps_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
