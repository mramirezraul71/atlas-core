"""Comprueba que Robot API (8002) responda. NEXUS consolidado en PUSH."""
from __future__ import annotations

import os
import urllib.request


def run() -> dict:
    robot_api = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
    timeout = int(os.getenv("NEXUS_TIMEOUT", "5"))
    robot_ok = False
    try:
        req = urllib.request.Request(robot_api + "/status", method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            robot_ok = r.status == 200
    except Exception:
        pass
    return {
        "ok": robot_ok,
        "check_id": "nexus_services_health",
        "message": "robot=" + ("ok" if robot_ok else "down"),
        "details": {"robot_api": robot_ok},
        "severity": "med" if not robot_ok else "low",
        "suggested_heals": ["restart_nexus_services"] if not robot_ok else [],
    }
