#!/usr/bin/env python3
"""Punto de entrada: Orquestador multi-agente. Reconexión NEXUS obligatoria antes de operar."""
from __future__ import annotations

import logging
import os
import sys
import time
import urllib.request
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))

ENV = BASE / "config" / "atlas.env"
if ENV.exists():
    from dotenv import load_dotenv
    load_dotenv(ENV, override=True)

# Integración Tríada: fuente de poder obligatoria para Evolution (daemon en segundo plano)
CREDENTIALS_MASTER = Path(r"C:\dev\credenciales.txt")
if not CREDENTIALS_MASTER.exists():
    os.environ.setdefault("EVOLUTION_CREDENTIALS_WARN", "1")

NEXUS_HEALTH_URL = (os.environ.get("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/") + "/health"
NEXUS_WAIT_TIMEOUT_SEC = int(os.environ.get("NEXUS_WAIT_TIMEOUT_SEC", "90"))
NEXUS_WAIT_POLL_SEC = 2

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
)
logger = logging.getLogger("main")


def _ping_nexus() -> bool:
    """True si NEXUS responde en /health."""
    try:
        req = urllib.request.Request(NEXUS_HEALTH_URL, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status == 200
    except Exception:
        return False


def wait_for_nexus(timeout_sec: int = NEXUS_WAIT_TIMEOUT_SEC) -> bool:
    """Bucle de monitoreo: espera hasta que NEXUS (cuerpo) responda. No permite operar sin confirmar conexión."""
    logger.info("CEREBRO — CUERPO (NEXUS): comprobando conexión en puerto 8000...")
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if _ping_nexus():
            logger.info("CEREBRO — CUERPO (NEXUS): Conectado. Iniciando orquestador.")
            return True
        time.sleep(NEXUS_WAIT_POLL_SEC)
    logger.error("CEREBRO — CUERPO (NEXUS): Desconectado tras %ss. El robot no puede operar sin el cuerpo. Ejecuta C:\\ATLAS_NEXUS\\scripts\\start_all.ps1", timeout_sec)
    return False


def main() -> None:
    logger.info("Iniciando sistema multi-agente (Orquestador)")
    if os.environ.get("EVOLUTION_CREDENTIALS_WARN") == "1":
        logger.info("Evolution: proceso crítico registrado en ANS (evolution_health). Daemon: python atlas_evolution.py")
    if not wait_for_nexus():
        sys.exit(1)
    # Prueba de Nervios: validar cámara y mouse; actualizar Dashboard a CONECTADO | ACTIVO
    try:
        import nexus_actions
        nexus_actions.run_nerve_test()
    except Exception as e:
        logger.debug("nerve_test: %s", e)
    try:
        from agents.orchestrator import run_orchestrator_cycle
        run_orchestrator_cycle()
    except KeyboardInterrupt:
        logger.info("Detenido por usuario")
    except Exception as e:
        logger.exception("main error: %s", e)
        raise


if __name__ == "__main__":
    main()
