"""Nervio: pies (movilidad).

Este módulo es la interfaz estable para controlar locomoción del robot.
Si no hay hardware/driver conectado, opera en modo stub (no ejecuta).
"""
from __future__ import annotations

import os
from typing import Any, Dict, Optional


def feet_status() -> Dict[str, Any]:
    """
    Estado del subsistema pies.
    Config:
      - FEET_DRIVER: "none" (default) | "nexus" | "sim" | "digital"
      - NEXUS_ENABLED + NEXUS_ATLAS_URL/NEXUS_ROBOT_API_URL (si driver=nexus)
    """
    driver = (os.getenv("FEET_DRIVER") or "none").strip().lower()
    configured = driver in ("nexus", "sim", "digital")
    web = None
    if driver == "digital":
        try:
            from modules.humanoid.web.navigator import status as web_status
            web = web_status()
        except Exception:
            web = {"enabled": False, "missing_deps": ["playwright"], "session_active": False}
    return {
        "ok": True,
        "configured": configured,
        "driver": driver,
        "note": None if configured else "feet driver not configured (stub mode)",
        "web": web,
    }


def feet_execute(command: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Ejecuta comando de movilidad.

    Comandos esperados (estándar):
      - move: {vx, vy, w, duration_ms}
      - stop: {}
      - step: {direction, steps}

    En stub devuelve ok=False con error=not_configured.
    """
    payload = payload or {}
    driver = (str(payload.get("driver") or "")).strip().lower() or (os.getenv("FEET_DRIVER") or "none").strip().lower()
    # Only allow explicit override to known safe drivers.
    if payload.get("driver") is not None and driver not in ("digital", "sim", "nexus", "none", "stub"):
        return {"ok": False, "error": "invalid_driver", "driver": driver}

    if driver in ("none", "", "stub"):
        return {"ok": False, "error": "not_configured", "driver": driver}

    if driver == "sim":
        # Simulador simple: no controla hardware, solo confirma recepción.
        return {"ok": True, "driver": driver, "command": (command or "").strip(), "payload": payload, "simulated": True}

    if driver == "nexus":
        # Placeholder para integración real (cuando exista API de motores en NEXUS).
        # Mantener seguro: no inventar endpoints no existentes.
        return {"ok": False, "error": "nexus_feet_api_not_implemented", "driver": driver}

    if driver == "digital":
        # Pies internos: navegación web automatizada (Playwright) bajo control del cerebro.
        try:
            from modules.humanoid.governance.state import get_emergency_stop
            if get_emergency_stop():
                return {"ok": False, "error": "emergency_stop", "driver": driver}
        except Exception:
            pass
        # Cargar bóveda (si existe) para credenciales web/API (sin pedir .env).
        try:
            from dotenv import load_dotenv
            vault_path = (os.getenv("ATLAS_VAULT_PATH") or r"C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt").strip()
            if os.path.isfile(vault_path):
                load_dotenv(vault_path, override=True)
            elif os.path.isfile(r"C:\dev\credenciales.txt"):
                load_dotenv(r"C:\dev\credenciales.txt", override=True)
        except Exception:
            pass
        cmd = (command or "").strip().lower()
        try:
            from modules.humanoid.web import navigator
            if cmd in ("workflow", "run", "steps"):
                steps = payload.get("steps") or []
                timeout_ms = int(payload.get("timeout_ms") or 30000)
                show_browser = payload.get("show_browser")
                return {"ok": True, "driver": driver, "result": navigator.run_steps(steps, timeout_ms=timeout_ms, show_browser=(None if show_browser is None else bool(show_browser)))}
            if cmd in ("open_url", "goto"):
                url = str(payload.get("url") or "")
                show_browser = payload.get("show_browser")
                return {"ok": True, "driver": driver, "result": navigator.open_url(url, timeout_ms=int(payload.get("timeout_ms") or 30000), show_browser=(None if show_browser is None else bool(show_browser)))}
            if cmd == "click":
                return {"ok": True, "driver": driver, "result": navigator.click(str(payload.get("selector") or ""), timeout_ms=int(payload.get("timeout_ms") or 5000))}
            if cmd == "fill":
                return {"ok": True, "driver": driver, "result": navigator.fill(str(payload.get("selector") or ""), str(payload.get("value") or ""), timeout_ms=int(payload.get("timeout_ms") or 5000))}
            if cmd in ("extract_text", "extract"):
                return {"ok": True, "driver": driver, "result": navigator.extract_text()}
            if cmd == "screenshot":
                return {"ok": True, "driver": driver, "result": navigator.screenshot(payload.get("path"))}
            if cmd in ("close", "reset"):
                navigator.close()
                return {"ok": True, "driver": driver, "result": {"ok": True}}
            return {"ok": False, "error": "unsupported_command", "driver": driver}
        except Exception as e:
            return {"ok": False, "error": str(e), "driver": driver}

    return {"ok": False, "error": "unknown_driver", "driver": driver}

