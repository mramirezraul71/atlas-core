"""
ATLAS DOCTOR DAEMON — Integración con el sistema humanoid/healing existente
============================================================================
Extiende el healing engine con los 15 puertos + 28 módulos SPA + ChromaDB.
Arranca como thread daemon desde lifespan.py o standalone.

Uso:
    python modules/humanoid/healing/atlas_doctor_daemon.py
    python modules/humanoid/healing/atlas_doctor_daemon.py --dry-run
"""
from __future__ import annotations

import asyncio
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

_log = logging.getLogger("atlas.doctor.daemon")

# Asegurar que BASE_DIR esté en sys.path
_BASE = Path(__file__).resolve().parent.parent.parent.parent
if str(_BASE) not in sys.path:
    sys.path.insert(0, str(_BASE))

# ── Puertos completos a monitorizar (15) ──────────────────────────────────────
MONITORED_PORTS = {
    8791: ("atlas_api",        "api",      "ATLAS API principal"),
    8795: ("quant_api",        "quant",    "ATLAS-Quant API"),
    8000: ("nexus_api",        "hardware", "NEXUS health gate"),
    8002: ("nexus_robot",      "hardware", "NEXUS robot backend"),
    8080: ("vision_espejo",    "vision",   "RAULI-VISION espejo.exe"),
    3000: ("vision_proxy",     "vision",   "RAULI-VISION Python proxy"),
    5174: ("vision_dashboard", "vision",   "RAULI-VISION dashboard"),
    3010: ("waha_whatsapp",    "comms",    "WAHA WhatsApp bridge"),
    3002: ("grafana",          "monitoring","Grafana"),
    9090: ("prometheus",       "monitoring","Prometheus"),
    # Puertos secundarios
    8792: ("quant_legacy",     "quant",    "Quant API legacy (puede estar libre)"),
}

# Módulos SPA dashboard que deben tener endpoint en el API
SPA_MODULE_CHECKS = [
    "/health",
    "/api/battery/status",
    "/api/brain/state",
    "/quality/pots/list",
    "/nervous/status",
    "/supervisor/daemon/status",
]

DAEMON_INTERVAL = int(os.getenv("ATLAS_DOCTOR_DAEMON_INTERVAL_SEC", "30"))
_DAEMON_RUNNING = False
_DAEMON_THREAD: Optional[threading.Thread] = None


# ── Integración con healing engine existente ───────────────────────────────────
def _record_to_healing_engine(component: str, ok: bool, detail: str) -> None:
    """Propaga eventos al audit logger del healing engine existente."""
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event(
            "doctor", "system", component, "health_check",
            ok, 0, None if ok else detail, {"detail": detail}, None
        )
    except Exception:
        pass


def _notify_healing_engine_on_crash(component: str, port: int) -> None:
    """Notifica al self-healing loop que un componente cayó."""
    try:
        # Registrar en ops_bus para que self_healing_loop también lo detecte
        from modules.humanoid.comms.ops_bus import emit as ops_emit
        ops_emit(
            "atlas.doctor.daemon",
            f"[DOCTOR DAEMON] Componente {component} (:{port}) caído — notificado a self-healing",
            level="WARNING",
            audio=False,
        )
    except Exception:
        pass


# ── Check de módulos SPA (verifica que los endpoints del API están activos) ────
def _check_spa_modules() -> dict:
    import socket
    import urllib.request

    results = {}
    if not _tcp_open("127.0.0.1", 8791):
        return {"atlas_api_down": True}

    for endpoint in SPA_MODULE_CHECKS:
        try:
            url = f"http://127.0.0.1:8791{endpoint}"
            req = urllib.request.Request(url, headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=1.5) as r:
                results[endpoint] = int(r.status)
        except Exception as e:
            results[endpoint] = f"ERROR: {e}"[:60]
    return results


def _tcp_open(host: str, port: int, timeout: float = 0.8) -> bool:
    import socket
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


# ── Check de ChromaDB (colecciones activas) ────────────────────────────────────
def _check_chromadb() -> dict:
    try:
        import chromadb
        base = _BASE
        chroma_path = base / "logs" / "chroma_db"
        if not chroma_path.exists():
            return {"status": "no_db_path"}
        client = chromadb.PersistentClient(path=str(chroma_path))
        cols = client.list_collections()
        return {
            "status": "ok",
            "collections": len(cols),
            "names": [c.name for c in cols[:10]],
        }
    except Exception as e:
        return {"status": "error", "error": str(e)[:80]}


# ── Ciclo principal del daemon ─────────────────────────────────────────────────
def _daemon_cycle() -> dict:
    """Un ciclo completo: puertos + SPA + ChromaDB + delega a AtlasDoctor."""
    report = {
        "ts": __import__("datetime").datetime.now(__import__("datetime").timezone.utc).isoformat(),
        "ports": {},
        "spa_modules": {},
        "chromadb": {},
        "doctor_cycle": {},
    }

    # 1. Puertos
    critical_down = []
    for port, (name, layer, desc) in MONITORED_PORTS.items():
        up = _tcp_open("127.0.0.1", port)
        report["ports"][name] = {"port": port, "up": up, "layer": layer}
        _record_to_healing_engine(name, up, f"TCP {port} {'UP' if up else 'DOWN'}")
        if not up and layer in ("api", "vision", "hardware"):
            critical_down.append((name, port))
            _notify_healing_engine_on_crash(name, port)

    # 2. SPA modules
    report["spa_modules"] = _check_spa_modules()

    # 3. ChromaDB
    report["chromadb"] = _check_chromadb()

    # 4. Delegar a AtlasDoctor (núcleo de detección + reparación)
    try:
        from atlas_adapter.services.doctor_nervous_system import get_doctor
        doctor = get_doctor()
        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(doctor.run_once())
            report["doctor_cycle"] = result
        finally:
            loop.close()
    except Exception as e:
        report["doctor_cycle"] = {"error": str(e)[:120]}

    if critical_down:
        _log.warning("[DOCTOR DAEMON] Críticos caídos: %s", critical_down)
    else:
        _log.debug("[DOCTOR DAEMON] Ciclo OK — %d puertos monitorizados", len(MONITORED_PORTS))

    return report


def _run_daemon_loop() -> None:
    global _DAEMON_RUNNING
    _log.info("[DOCTOR DAEMON] Iniciado (interval=%ss)", DAEMON_INTERVAL)
    while _DAEMON_RUNNING:
        try:
            _daemon_cycle()
        except Exception as e:
            _log.error("[DOCTOR DAEMON] Error en ciclo: %s", e)
        time.sleep(DAEMON_INTERVAL)
    _log.info("[DOCTOR DAEMON] Detenido")


# ── API pública del daemon ──────────────────────────────────────────────────────
def start_doctor_daemon() -> bool:
    global _DAEMON_RUNNING, _DAEMON_THREAD
    if _DAEMON_RUNNING:
        return True
    _DAEMON_RUNNING = True
    _DAEMON_THREAD = threading.Thread(
        target=_run_daemon_loop,
        name="atlas-doctor-daemon",
        daemon=True,
    )
    _DAEMON_THREAD.start()
    return True


def stop_doctor_daemon() -> None:
    global _DAEMON_RUNNING
    _DAEMON_RUNNING = False


def is_doctor_running() -> bool:
    return _DAEMON_RUNNING and (_DAEMON_THREAD is not None) and _DAEMON_THREAD.is_alive()


def get_daemon_status() -> dict:
    return {
        "running": is_doctor_running(),
        "thread": _DAEMON_THREAD.name if _DAEMON_THREAD else None,
        "interval_sec": DAEMON_INTERVAL,
        "monitored_ports": len(MONITORED_PORTS),
        "spa_endpoints": len(SPA_MODULE_CHECKS),
    }


# ── Entrada standalone ─────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="ATLAS DOCTOR DAEMON")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--once", action="store_true", help="Un ciclo y salir")
    args = parser.parse_args()

    if args.dry_run:
        os.environ["ATLAS_DOCTOR_DRY_RUN"] = "true"

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s"
    )

    if args.once:
        result = _daemon_cycle()
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        start_doctor_daemon()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            stop_doctor_daemon()
            print("\n[DOCTOR DAEMON] Detenido.")
