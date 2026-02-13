"""Disk health check."""
from __future__ import annotations

import os
import shutil


def run() -> dict:
    try:
        base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
        usage = shutil.disk_usage(base)
        free_gb = usage.free / (1024 ** 3)
        ok = free_gb > 1.0
        return {"ok": ok, "check_id": "disk_health", "message": f"free={free_gb:.1f}GB", "details": {"free_gb": round(free_gb, 2)}, "severity": "high" if free_gb < 0.5 else ("med" if not ok else "low"), "suggested_heals": ["rotate_logs"] if not ok else []}
    except Exception as e:
        return {"ok": False, "check_id": "disk_health", "message": str(e), "details": {"error": str(e)}, "severity": "low"}
