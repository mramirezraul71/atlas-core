"""Logs health: error spam detection."""
from __future__ import annotations

import os
from pathlib import Path


def run() -> dict:
    try:
        base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
        log_dir = Path(base) / "logs"
        if not log_dir.exists():
            return {"ok": True, "check_id": "logs_health", "message": "no logs", "details": {}, "severity": "low"}
        last_lines = []
        for p in sorted(log_dir.glob("*.log"), key=lambda x: x.stat().st_mtime, reverse=True)[:2]:
            try:
                with open(p, "r", encoding="utf-8", errors="ignore") as f:
                    lines = f.readlines()
                    last_lines.extend(lines[-50:])
            except Exception:
                pass
        errors = [l for l in last_lines if "error" in l.lower() or "exception" in l.lower() or "traceback" in l.lower()]
        spam = len([e for e in errors if errors.count(e) >= 5]) > 0 if errors else False
        ok = not spam
        return {"ok": ok, "check_id": "logs_health", "message": f"errors={len(errors)} spam={spam}", "details": {"error_count": len(errors)}, "severity": "med" if spam else "low", "suggested_heals": ["rotate_logs"] if spam else []}
    except Exception as e:
        return {"ok": True, "check_id": "logs_health", "message": str(e), "details": {"error": str(e)}, "severity": "low"}
