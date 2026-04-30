from __future__ import annotations

import logging
from typing import Optional

_log = logging.getLogger("atlas.compat.daemon")

MONITORED_PORTS = {
    8791: ("atlas_api", "api", "ATLAS API principal"),
    8795: ("quant_api", "quant", "ATLAS-Quant API"),
    8000: ("nexus_api", "hardware", "NEXUS health gate"),
    8002: ("nexus_robot", "hardware", "NEXUS robot backend"),
}

_DAEMON_RUNNING = False
_DAEMON_THREAD: Optional[object] = None


def _check_chromadb() -> dict:
    return {"status": "disabled", "reason": "atlas_doctor_removed"}


def start_doctor_daemon() -> bool:
    _log.info("[DOCTOR DAEMON] deshabilitado: el modulo Doctor fue eliminado")
    return False


def stop_doctor_daemon() -> None:
    return None


def is_doctor_running() -> bool:
    return False


def get_daemon_status() -> dict:
    return {
        "enabled": False,
        "running": False,
        "reason": "atlas_doctor_removed",
        "monitored_ports": MONITORED_PORTS,
    }

