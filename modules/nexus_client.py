"""NEXUS client — consume APIs de ATLAS NEXUS (robot, visión, directivas).
Usado por PUSH para unificar panel: cerebro + robot.
Incluye retry con backoff corto para resiliencia (prompt Claude NEXUS).
"""
import os
import logging
import time
import urllib.request
import urllib.error
import json
from typing import Optional, Any

logger = logging.getLogger(__name__)

NEXUS_BASE_URL = os.getenv("NEXUS_BASE_URL", "").rstrip("/")
NEXUS_ENABLED = os.getenv("NEXUS_ENABLED", "false").strip().lower() in ("1", "true", "yes", "y", "on")
NEXUS_TIMEOUT = int(os.getenv("NEXUS_TIMEOUT", "5"))

# Backoff en segundos para reintentos (resiliencia handshake PUSH-NEXUS)
NEXUS_RETRY_DELAYS = [2, 4, 6]  # 3 reintentos = hasta 4 intentos

NEXUS_ROBOT_URL = (os.getenv("NEXUS_ROBOT_URL") or "http://127.0.0.1:5174").rstrip("/")
NEXUS_ROBOT_API_URL = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")


def _fetch(url: str) -> Optional[dict]:
    """GET JSON desde URL. Retorna None si falla."""
    try:
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=NEXUS_TIMEOUT) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        logger.debug("NEXUS fetch %s: %s", url, e)
        return None


def _fetch_with_retry(url: str, max_attempts: int = 1 + len(NEXUS_RETRY_DELAYS)) -> Optional[dict]:
    """GET JSON con reintentos y backoff. Para handshake resiliente con NEXUS."""
    for attempt in range(max_attempts):
        out = _fetch(url)
        if out is not None:
            return out
        if attempt < len(NEXUS_RETRY_DELAYS):
            delay = NEXUS_RETRY_DELAYS[attempt]
            logger.debug("NEXUS retry %s en %ss: %s", attempt + 1, delay, url)
            time.sleep(delay)
    return None


def _probe_ok(url: str) -> bool:
    """Comprueba si la URL responde 200 (cuerpo, frontend HTML)."""
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=NEXUS_TIMEOUT) as resp:
            return resp.status == 200
    except Exception as e:
        logger.debug("Probe %s: %s", url, e)
        return False


def _scan_body() -> dict:
    """Escanea el cuerpo: dashboard Robot (5174) y API cámaras (8002)."""
    dashboard_ok = _probe_ok(NEXUS_ROBOT_URL + "/") if NEXUS_ENABLED else False
    api_status = _fetch(NEXUS_ROBOT_API_URL + "/api/camera/service/status") if NEXUS_ENABLED else None
    api_health = _fetch(NEXUS_ROBOT_API_URL + "/api/health") if NEXUS_ENABLED else None
    api_ok = api_status is not None or api_health is not None
    return {
        "dashboard": {"ok": dashboard_ok, "url": NEXUS_ROBOT_URL},
        "api_cameras": {"ok": api_ok, "url": NEXUS_ROBOT_API_URL, "status": api_status or api_health},
    }


def get_nexus_status() -> dict:
    """Estado de NEXUS (robot). Si NEXUS no está configurado o no responde, retorna disconnected."""
    if not NEXUS_ENABLED or not NEXUS_BASE_URL:
        return {
            "ok": False,
            "connected": False,
            "reason": "NEXUS_ENABLED=false o NEXUS_BASE_URL vacío",
            "nexus": None,
            "directives": None,
            "vision": None,
        }
    base = NEXUS_BASE_URL.rstrip("/")
    # Primer contacto con retry+backoff para resiliencia (NEXUS puede tardar en arrancar)
    status = _fetch_with_retry(f"{base}/status")
    if status is None:
        status = _fetch_with_retry(f"{base}/health")
    directives = _fetch(f"{base}/directives/summary")
    directives_health = _fetch(f"{base}/directives/health")
    vision = _fetch(f"{base}/api/vision/status") or _fetch(f"{base}/vision/status")

    connected = bool(status or directives or directives_health)
    body_scan = _scan_body() if NEXUS_ENABLED else {}
    return {
        "ok": connected,
        "connected": connected,
        "nexus_base_url": base,
        "nexus": status,
        "directives": directives or directives_health,
        "directives_health": directives_health,
        "vision": vision,
        "body_scan": body_scan,
        "reason": None if connected else "NEXUS no responde en " + base,
    }
