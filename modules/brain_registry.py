"""
ATLAS Brain Registry — Inventario y dominio del cerebro sobre todas las piezas del robot.

El cerebro (PUSH) CONOCE qué tiene y ejerce su dominio sobre cada parte.
Este módulo define el ensamblado completo: inventario, rutas de control, salud.
"""

import os
import logging
import urllib.request
import urllib.error
import json
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

# =========================
# Configuración desde env
# =========================
NEXUS_BASE_URL = (os.getenv("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/")
NEXUS_ROBOT_URL = (os.getenv("NEXUS_ROBOT_URL") or "http://127.0.0.1:5174").rstrip("/")
NEXUS_ROBOT_API_URL = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
NEXUS_ENABLED = os.getenv("NEXUS_ENABLED", "false").strip().lower() in ("1", "true", "yes", "y", "on")
PROBE_TIMEOUT = int(os.getenv("NEXUS_TIMEOUT", "5"))


def _probe(url: str, expect_json: bool = True) -> Optional[dict]:
    """GET desde URL. Si expect_json=True, retorna dict o None. Si False, retorna {'ok': True} si 200."""
    try:
        headers = {"Accept": "application/json"} if expect_json else {}
        req = urllib.request.Request(url, method="GET", headers=headers)
        with urllib.request.urlopen(req, timeout=PROBE_TIMEOUT) as resp:
            if not expect_json:
                return {"ok": True, "status_code": resp.status} if resp.status == 200 else None
            data = resp.read().decode("utf-8")
            return json.loads(data) if data.strip() else {}
    except Exception as e:
        logger.debug("Probe %s: %s", url, e)
        return None


# =========================
# INVENTARIO — Partes que el cerebro controla
# =========================

@dataclass
class PartDef:
    """Definición de una pieza del robot bajo dominio del cerebro."""
    id: str
    name: str
    role: str  # cerebro | directivas | cuerpo | sensores | memoria | comunicacion
    url: str
    probe_path: str
    enabled: bool
    control_route: str  # ruta en PUSH para enviar comandos
    description: str = ""


def get_inventory() -> List[Dict[str, Any]]:
    """Inventario de todas las piezas que el cerebro conoce y controla."""
    base = os.getenv("ATLAS_BASE_URL", "http://127.0.0.1:8791").rstrip("/")

    parts = [
        {
            "id": "cerebro",
            "name": "PUSH (Cerebro)",
            "role": "cerebro",
            "url": base,
            "probe_path": "/health",
            "enabled": True,
            "control_route": "/status, /intent, /agent/goal, /execute",
            "description": "Orquestador central: aprobaciones, memoria, ANS, scheduler, policy, audit",
        },
        {
            "id": "nexus",
            "name": "NEXUS (Directivas + Multi-IA)",
            "role": "directivas",
            "url": NEXUS_BASE_URL,
            "probe_path": "/status",
            "enabled": NEXUS_ENABLED,
            "control_route": f"{base}/robot/* → proxy a NEXUS",
            "description": "Directivas, goals, think, neural router (Ollama/DeepSeek/Claude/GPT), 50+ tools",
        },
        {
            "id": "cuerpo_frontend",
            "name": "Robot Dashboard (Cámaras, Visión)",
            "role": "cuerpo",
            "url": NEXUS_ROBOT_URL,
            "probe_path": "/",
            "expect_json": False,
            "enabled": NEXUS_ENABLED,
            "control_route": f"{base}/cuerpo/* → proxy a Robot frontend",
            "description": "UI: visión, YOLO, simulación, brain chat, identity",
        },
        {
            "id": "cuerpo_backend",
            "name": "Robot API (Visión, Cámaras)",
            "role": "sensores",
            "url": NEXUS_ROBOT_API_URL,
            "probe_path": "/api/camera/service/status",
            "enabled": NEXUS_ENABLED,
            "control_route": f"{base}/cuerpo/api/* → Robot API 8002",
            "description": "API cámaras, detección, stream, YOLO, network cameras",
        },
    ]
    return parts


def probe_part(part: Dict[str, Any]) -> Dict[str, Any]:
    """Prueba una pieza y retorna estado."""
    url = part.get("url", "").rstrip("/")
    path = part.get("probe_path", "/").lstrip("/")
    probe_url = f"{url}/{path}" if path and path != "/" else url
    expect_json = part.get("expect_json", True)
    result = _probe(probe_url, expect_json=expect_json)
    ok = result is not None
    return {
        "id": part["id"],
        "ok": ok,
        "status": result if ok else None,
        "error": None if ok else "No responde",
    }


def get_assembly_status() -> Dict[str, Any]:
    """
    Estado del ensamblado completo.
    El cerebro sabe qué tiene, qué está vivo y cómo ejercer dominio.
    """
    inventory = get_inventory()
    probes = {}
    for part in inventory:
        if part.get("enabled"):
            probes[part["id"]] = probe_part(part)
        else:
            probes[part["id"]] = {"id": part["id"], "ok": False, "status": None, "error": "Deshabilitado"}

    # Rutas de control: a dónde enviar cada tipo de comando
    control_routes = {
        "comando_rapido": "/intent",
        "objetivo_ia": "/agent/goal",
        "ejecutar_paso": "/agent/step/execute",
        "directivas": "/robot/directives/*",
        "goals_nexus": "/robot/goal",
        "think_nexus": "/robot/think",
        "vision_stream": "/cuerpo/",
        "camaras_api": "/cuerpo/api/camera/*",
        "status_nexus": "/nexus/status",
        "voice_approval": "/owner/voice/command",
    }

    return {
        "ok": True,
        "brain": "ATLAS PUSH",
        "message": "Inventario y dominio del cerebro sobre todas las piezas",
        "inventory": inventory,
        "probes": probes,
        "control_routes": control_routes,
        "summary": {
            "total_parts": len(inventory),
            "enabled": sum(1 for p in inventory if p.get("enabled")),
            "alive": sum(1 for r in probes.values() if r.get("ok")),
        },
    }
