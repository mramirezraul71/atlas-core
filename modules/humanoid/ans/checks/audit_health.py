"""Audit health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.audit import get_audit_logger
        logger = get_audit_logger()
        if getattr(logger, "_db", None) is None:
            return {"ok": False, "check_id": "audit_health", "message": "AUDIT_DB_PATH not set", "details": {}, "severity": "med"}
        conn = logger._db._ensure_conn()
        conn.execute("SELECT 1")
        conn.commit()
        return {"ok": True, "check_id": "audit_health", "message": "ok", "details": {}, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "audit_health", "message": str(e), "details": {"error": str(e)}, "severity": "med", "suggested_heals": []}
