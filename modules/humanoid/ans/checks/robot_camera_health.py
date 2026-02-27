"""Comprueba que el servicio de cámaras del Robot responda rápido (cuando NEXUS_ENABLED)."""

from __future__ import annotations

import os
import urllib.request


def run() -> dict:
    if os.getenv("NEXUS_ENABLED", "").strip().lower() not in ("1", "true", "yes", "y", "on"):
        return {"ok": True, "check_id": "robot_camera_health", "message": "NEXUS disabled", "severity": "low", "suggested_heals": []}
    robot_api = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
    timeout = int(os.getenv("NEXUS_TIMEOUT", "5"))
    try:
        # fast=true evita detección lenta
        req = urllib.request.Request(robot_api + "/api/camera/service/status?fast=true", method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            ok = r.status == 200
            return {
                "ok": ok,
                "check_id": "robot_camera_health",
                "message": "camera_service=" + ("ok" if ok else "down"),
                "details": {"status_code": r.status},
                "severity": "med" if not ok else "low",
                "suggested_heals": ["restart_nexus_services"] if not ok else [],
            }
    except Exception as e:
        return {
            "ok": False,
            "check_id": "robot_camera_health",
            "message": f"camera_service down: {str(e)[:160]}",
            "details": {"error": str(e)},
            "severity": "med",
            "suggested_heals": ["restart_nexus_services"],
        }

