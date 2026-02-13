"""Memory health check."""
from __future__ import annotations

import os


def run() -> dict:
    if os.getenv("MEMORY_ENABLED", "true").strip().lower() not in ("1", "true", "yes"):
        return {"ok": True, "check_id": "memory_health", "message": "memory disabled", "details": {}, "severity": "low"}
    try:
        from modules.humanoid.memory_engine import db as mem_db
        conn = mem_db._ensure()
        conn.execute("SELECT 1")
        conn.commit()
        return {"ok": True, "check_id": "memory_health", "message": "ok", "details": {}, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "memory_health", "message": str(e), "details": {"error": str(e)}, "severity": "med", "suggested_heals": []}
