from __future__ import annotations

import socket
import urllib.request
from typing import Dict, Tuple

from ..models import POT, POTSeverity, POTStep


def _tcp_open(host: str, port: int, timeout: float = 0.8) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def _http_get(url: str, timeout: float = 1.5) -> Tuple[bool, str]:
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", "replace")
            return (200 <= int(r.status) < 300), raw[:500]
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        host = "127.0.0.1"
        ports = {
            "atlas_api_adapter": 8791,
            "nexus_api": 8000,
            "robot_backend": 8002,
            "waha_whatsapp": 3010,
        }

        lines = ["SERVICES / PORTS CHECK (check-only)"]
        for name, port in ports.items():
            up = _tcp_open(host, port)
            lines.append(f"- {name}: {'UP' if up else 'DOWN'} (tcp {host}:{port})")

        # Endpoints (si el adapter está up)
        ok_status, body = _http_get("http://127.0.0.1:8791/status")
        lines.append(f"- GET /status: {'OK' if ok_status else 'FAIL'}")
        if body:
            lines.append(f"  body={body}")

        lines.append("")
        lines.append("Acciones recomendadas (manuales):")
        lines.append("- Levantar API Adapter: ejecutar `03_run_atlas_api.ps1` (puerto 8791).")
        lines.append("- Verificar NEXUS: confirmar proceso y `/health` en 8000.")
        lines.append("- Verificar Robot Backend: confirmar proceso y `/health` en 8002.")
        lines.append("- WhatsApp (WAHA): confirmar contenedor/servicio en 3010 si se usa.")

        return "\n".join(lines)

    return POT(
        id="services_ports_check",
        name="Services/Ports Check (8791/8000/8002/3010)",
        description="Audita puertos y endpoints básicos (TCP + /status). No reinicia nada.",
        severity=POTSeverity.MED,
        rules=[
            "Prohibido reiniciar servicios en automático en este POT.",
            "Solo evidencia: TCP y HTTP GET.",
        ],
        tags=["services", "ports", "nexus", "robot", "waha"],
        steps=[POTStep(id="ports", name="Chequear puertos y /status", run=_run, fatal=True)],
    )

