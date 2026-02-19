"""Comprueba que Robot API (8002) responda. NEXUS consolidado en PUSH."""
from __future__ import annotations

import os
import time
import urllib.request

# Reintentos antes de declarar robot caído: evita falsos positivos
# por timeout durante inferencia pesada (YOLO, TF, etc.)
_RETRIES = int(os.getenv("NEXUS_HEALTH_RETRIES", "3"))
_RETRY_DELAY = float(os.getenv("NEXUS_HEALTH_RETRY_DELAY", "2"))
_TIMEOUT = int(os.getenv("NEXUS_TIMEOUT", "12"))


def _probe(url: str, timeout: int) -> bool:
    try:
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


def run() -> dict:
    robot_api = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")

    # Intentar con el endpoint /api/health primero, luego /status como fallback
    endpoints = [robot_api + "/api/health", robot_api + "/status"]
    robot_ok = False

    for attempt in range(_RETRIES):
        for ep in endpoints:
            if _probe(ep, _TIMEOUT):
                robot_ok = True
                break
        if robot_ok:
            break
        if attempt < _RETRIES - 1:
            time.sleep(_RETRY_DELAY)

    return {
        "ok": robot_ok,
        "check_id": "nexus_services_health",
        "message": "robot=" + ("ok" if robot_ok else "down"),
        "details": {"robot_api": robot_ok, "retries": _RETRIES, "timeout_s": _TIMEOUT},
        "severity": "med" if not robot_ok else "low",
        "suggested_heals": ["restart_nexus_services"] if not robot_ok else [],
    }
