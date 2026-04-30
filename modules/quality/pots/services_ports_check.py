from __future__ import annotations

import socket
import urllib.request
from typing import Dict, List, Tuple

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
            return (200 <= int(r.status) < 300), raw[:400]
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def _section(title: str) -> str:
    return f"\n{'─'*60}\n  {title}\n{'─'*60}"


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        host = "127.0.0.1"

        # ── Puertos TCP a verificar ──────────────────────────────────────────
        #  (nombre, puerto, descripcion, fix_hint)
        port_matrix: List[Tuple[str, int, str, str]] = [
            # Core ATLAS
            ("atlas_api_adapter",    8791, "ATLAS API principal (FastAPI/uvicorn)",    "03_run_atlas_api.ps1"),
            ("nexus_health",         8000, "NEXUS robot — health gate",                "nexus/atlas_nexus/run.py"),
            ("robot_backend",        8002, "NEXUS robot backend secundario",           "atlas_nexus_robot"),
            # Comunicaciones
            ("waha_whatsapp",        3010, "WAHA — WhatsApp API Bridge",               "docker compose up waha"),
            # ATLAS-Quant
            ("quant_api_8795",       8795, "ATLAS-Quant API (activo, post-migración)",  "atlas_quant_start.ps1 -Port 8795"),
            ("quant_api_8792",       8792, "ATLAS-Quant API (puerto legacy)",           "puede estar libre"),
            # RAULI-VISION
            ("vision_espejo_go",     8080, "Espejo Go backend (espejo.exe)",            "RAULI_VISION.ps1"),
            ("vision_proxy_python",  3000, "Proxy Python → espejo (Cloudflare apunta aquí)", "RAULI_VISION.ps1"),
            ("vision_dashboard",     5174, "Dashboard React/Vite (RAULI-VISION UI)",   "RAULI_VISION.ps1"),
            # Monitoring (opcional)
            ("grafana",              3003, "Grafana — métricas y dashboards",           "start_monitoring.ps1"),
            ("prometheus",           9090, "Prometheus — metrics scraper",              "start_monitoring.ps1"),
        ]

        lines: List[str] = []
        lines.append("SERVICES / PORTS CHECK v2.0")
        lines.append(f"Timestamp: {__import__('datetime').datetime.now().isoformat()}")

        # ── Sección 1: TCP ───────────────────────────────────────────────────
        lines.append(_section("PUERTOS TCP"))
        critical_down = []
        for name, port, desc, fix in port_matrix:
            up = _tcp_open(host, port)
            status = "UP  " if up else "DOWN"
            lines.append(f"  [{status}] :{port:<5}  {name:<28}  {desc}")
            if not up and port in (8791, 8080, 3000):
                critical_down.append((name, port, fix))

        # ── Sección 2: HTTP endpoints clave ─────────────────────────────────
        lines.append(_section("HTTP ENDPOINTS"))
        endpoints = [
            ("ATLAS /status",              "http://127.0.0.1:8791/status"),
            ("ATLAS /health",              "http://127.0.0.1:8791/health"),
            ("ATLAS /api/battery/status",  "http://127.0.0.1:8791/api/battery/status"),
            ("Quant /health",              "http://127.0.0.1:8795/health"),
            ("Quant /operation/loop/status","http://127.0.0.1:8795/operation/loop/status"),
            ("Quant /paper/account",       "http://127.0.0.1:8795/paper/account"),
            ("Vision proxy /api/health",   "http://127.0.0.1:3000/api/health"),
            ("Espejo /api/health",         "http://127.0.0.1:8080/api/health"),
            ("NEXUS /health",              "http://127.0.0.1:8000/health"),
        ]
        for label, url in endpoints:
            ok, body = _http_get(url)
            mark = "OK  " if ok else "FAIL"
            snippet = body[:80].replace("\n", " ") if body else ""
            lines.append(f"  [{mark}] {label:<35}  {snippet}")

        # ── Sección 3: Correcciones recomendadas ─────────────────────────────
        lines.append(_section("ACCIONES RECOMENDADAS"))
        if critical_down:
            lines.append("  CRÍTICOS (afectan Cloudflare tunnel y funcionalidad core):")
            for name, port, fix in critical_down:
                lines.append(f"    - {name} :{port} → {fix}")
        else:
            lines.append("  Sin servicios críticos caídos.")

        lines.append("")
        lines.append("  Referencia general:")
        lines.append("    - ATLAS API        : .\\03_run_atlas_api.ps1  (puerto 8791)")
        lines.append("    - NEXUS Robot      : confirmar proceso en :8000/:8002")
        lines.append("    - ATLAS-Quant      : .\\scripts\\atlas_quant_start.ps1 -Port 8795")
        lines.append("    - RAULI-VISION     : .\\RAULI_VISION.ps1  (8080 + 3000 + 5174)")
        lines.append("    - Monitoring       : .\\tools\\prometheus\\start_monitoring.ps1")
        lines.append("    - WhatsApp (WAHA)  : docker compose up waha  (puerto 3010)")

        return "\n".join(lines)

    return POT(
        id="services_ports_check",
        name="Services/Ports Check — todos los módulos ATLAS",
        description=(
            "Audita puertos TCP y endpoints HTTP de todos los módulos activos: "
            "Core (8791/8000/8002), Quant (8795), RAULI-VISION (8080/3000/5174), "
            "Monitoring (3003/9090), WhatsApp (3010). Solo lectura — no reinicia nada."
        ),
        severity=POTSeverity.MED,
        rules=[
            "Prohibido reiniciar servicios en automático en este POT.",
            "Solo evidencia: TCP connect + HTTP GET.",
            "Reporte de correcciones es orientativo — el operador decide.",
        ],
        tags=["services", "ports", "nexus", "robot", "quant", "vision", "cloudflare", "waha"],
        steps=[
            POTStep(id="ports_http", name="Verificar puertos TCP y endpoints HTTP", run=_run, fatal=True)
        ],
    )
