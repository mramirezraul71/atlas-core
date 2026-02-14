"""Comprueba que NEXUS (8000) y Robot API (8002) respondan cuando NEXUS_ENABLED."""
from __future__ import annotations

import os
import urllib.request


def run() -> dict:
    if os.getenv("NEXUS_ENABLED", "").strip().lower() not in ("1", "true", "yes", "y", "on"):
        return {"ok": True, "check_id": "nexus_services_health", "message": "NEXUS disabled", "severity": "low", "suggested_heals": []}
    base = (os.getenv("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/")
    robot_api = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
    timeout = int(os.getenv("NEXUS_TIMEOUT", "5"))
    issues = []
    nexus_ok = False
    robot_ok = False
    try:
        req = urllib.request.Request(base + "/status", method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            nexus_ok = r.status == 200
    except Exception:
        issues.append("NEXUS (8000) no responde")
    try:
        req = urllib.request.Request(robot_api + "/api/health", method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            robot_ok = r.status == 200
    except Exception:
        try:
            req = urllib.request.Request(robot_api + "/status", method="GET", headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                robot_ok = r.status == 200
        except Exception:
            issues.append("Robot API (8002) no responde")
    ok = nexus_ok and robot_ok
    return {
        "ok": ok,
        "check_id": "nexus_services_health",
        "message": "nexus=" + ("ok" if nexus_ok else "down") + " robot=" + ("ok" if robot_ok else "down"),
        "details": {"nexus": nexus_ok, "robot_api": robot_ok},
        "severity": "med" if not ok else "low",
        "suggested_heals": ["restart_nexus_services"] if not ok else [],
    }
