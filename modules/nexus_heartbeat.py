"""Heartbeat NEXUS: ping constante a 127.0.0.1:8000/health. Auto-reactivación con start_all.ps1."""
from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from typing import Callable, Optional

logger = logging.getLogger("atlas.nexus_heartbeat")

NEXUS_HEALTH_URL = os.getenv("NEXUS_BASE_URL", "http://127.0.0.1:8000").rstrip("/") + "/health"
NEXUS_HEARTBEAT_INTERVAL_SEC = float(os.getenv("NEXUS_HEARTBEAT_INTERVAL_SEC", "15"))
NEXUS_START_SCRIPT = os.getenv("NEXUS_START_SCRIPT", r"C:\ATLAS_NEXUS\scripts\start_all.ps1")
NEXUS_TIMEOUT = int(os.getenv("NEXUS_TIMEOUT", "5"))

_nexus_connected = False
_nexus_active = False  # True cuando la Prueba de Nervios (mouse/cámara) ha sido exitosa
_nexus_last_check_ts: float = 0
_nexus_last_error: str = ""
_heartbeat_thread: Optional[threading.Thread] = None
_heartbeat_stop = threading.Event()
_on_status_change: Optional[Callable[[bool, str], None]] = None


def ping_nexus() -> tuple[bool, str]:
    """GET NEXUS_HEALTH_URL. Devuelve (ok, mensaje)."""
    try:
        req = urllib.request.Request(NEXUS_HEALTH_URL, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=NEXUS_TIMEOUT) as r:
            ok = r.status == 200
            msg = "OK" if ok else f"HTTP {r.status}"
            return (ok, msg)
    except Exception as e:
        return (False, str(e)[:120])


def restart_nexus() -> bool:
    """Ejecuta NEXUS_START_SCRIPT o arranca NEXUS desde el repo (start.ps1 o python). Devuelve True si se lanzó."""
    base = Path(__file__).resolve().parent.parent  # ATLAS_PUSH
    script_candidates = [
        Path(NEXUS_START_SCRIPT),
        base / "scripts" / "start_nexus.ps1",
    ]
    for path in script_candidates:
        if path.exists():
            try:
                subprocess.Popen(
                    ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(path)],
                    cwd=str(base),
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                )
                logger.info("Auto-reactivación NEXUS: lanzado %s", path)
                return True
            except Exception as e:
                logger.exception("Error al ejecutar %s: %s", path, e)
                continue
    # Fallback: start.ps1 dentro de nexus/atlas_nexus (activa venv)
    nexus_dir = Path(os.getenv("NEXUS_ATLAS_PATH") or str(base / "nexus" / "atlas_nexus"))
    start_ps1 = nexus_dir / "start.ps1"
    if start_ps1.exists():
        try:
            subprocess.Popen(
                ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(start_ps1), "-mode", "api"],
                cwd=str(nexus_dir),
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
            logger.info("Auto-reactivación NEXUS: lanzado start.ps1 en %s", nexus_dir)
            return True
        except Exception as e:
            logger.debug("start.ps1 fallback: %s", e)
    # Último fallback: python nexus.py
    nexus_py = nexus_dir / "nexus.py"
    if nexus_py.exists():
        try:
            subprocess.Popen(
                [os.environ.get("PYTHON", "python"), "nexus.py", "--mode", "api"],
                cwd=str(nexus_dir),
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
            logger.info("Auto-reactivación NEXUS: python nexus.py --mode api en %s", nexus_dir)
            return True
        except Exception as e:
            logger.debug("python fallback: %s", e)
    logger.warning("NEXUS no arrancado: ningún script en repo ni NEXUS_START_SCRIPT=%s", NEXUS_START_SCRIPT)
    return False


def get_nexus_connection_state() -> dict:
    """Estado actual: connected, active, last_check_ts, last_error."""
    return {
        "connected": _nexus_connected,
        "active": _nexus_active,
        "last_check_ts": _nexus_last_check_ts,
        "last_error": _nexus_last_error,
    }


def set_nexus_active(active: bool) -> None:
    """Marca cuerpo como ACTIVO (Prueba de Nervios exitosa)."""
    global _nexus_active
    _nexus_active = active


def set_nexus_connected(connected: bool, message: str = "") -> None:
    """Actualiza estado y opcionalmente notifica (POST Dashboard / bitácora)."""
    global _nexus_connected, _nexus_last_check_ts, _nexus_last_error
    _nexus_connected = connected
    _nexus_last_check_ts = time.time()
    _nexus_last_error = message if not connected else ""
    if _on_status_change:
        try:
            _on_status_change(connected, message)
        except Exception:
            pass


def register_status_callback(callback: Callable[[bool, str], None]) -> None:
    """Registra callback al cambiar estado (conectado/desconectado). Para enviar a Bitácora/Dashboard."""
    global _on_status_change
    _on_status_change = callback


def _heartbeat_loop() -> None:
    """Bucle: ping cada NEXUS_HEARTBEAT_INTERVAL_SEC; si falla, intenta restart_nexus()."""
    global _nexus_connected
    consecutive_failures = 0
    while not _heartbeat_stop.wait(timeout=NEXUS_HEARTBEAT_INTERVAL_SEC):
        ok, msg = ping_nexus()
        if ok:
            consecutive_failures = 0
            if not _nexus_connected:
                set_nexus_connected(True, "")
                logger.info("[CONEXIÓN] NEXUS en puerto 8000... OK.")
        else:
            consecutive_failures += 1
            set_nexus_connected(False, msg)
            if consecutive_failures >= 2:
                logger.warning("[CONEXIÓN] NEXUS no responde. Auto-reactivación: %s", NEXUS_START_SCRIPT)
                restart_nexus()
                consecutive_failures = 0


def start_heartbeat() -> None:
    """Inicia el hilo de heartbeat. Idempotente. Primer ping; si falla, intenta arrancar NEXUS una vez."""
    global _heartbeat_thread
    if _heartbeat_thread is not None and _heartbeat_thread.is_alive():
        return
    ok, msg = ping_nexus()
    set_nexus_connected(ok, "" if ok else msg)
    if not ok:
        logger.info("NEXUS no responde en 8000. Intentando arrancar una vez...")
        restart_nexus()
    _heartbeat_stop.clear()
    _heartbeat_thread = threading.Thread(target=_heartbeat_loop, name="nexus-heartbeat", daemon=True)
    _heartbeat_thread.start()
    logger.info("Heartbeat NEXUS iniciado (intervalo %ss). Primer ping: %s", NEXUS_HEARTBEAT_INTERVAL_SEC, "OK" if ok else msg)


def stop_heartbeat() -> None:
    """Detiene el hilo de heartbeat."""
    _heartbeat_stop.set()


def reconnect_nexus_and_poll(wait_after_start_sec: float = 6, poll_interval_sec: float = 2, max_polls: int = 15) -> dict:
    """
    Intenta arrancar NEXUS (restart_nexus), espera wait_after_start_sec y luego hace ping cada poll_interval_sec
    hasta max_polls. Actualiza el estado global y devuelve get_nexus_connection_state().
    """
    restart_nexus()
    time.sleep(wait_after_start_sec)
    for _ in range(max_polls):
        ok, msg = ping_nexus()
        if ok:
            set_nexus_connected(True, "")
            return get_nexus_connection_state()
        time.sleep(poll_interval_sec)
    set_nexus_connected(False, msg)
    return get_nexus_connection_state()
