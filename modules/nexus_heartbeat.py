"""Heartbeat NEXUS: ping constante a 127.0.0.1:8000/health. Auto-reactivación con start_all.ps1.
Integrado con ATLAS AUTONOMOUS: HealthAggregator, CircuitBreaker, HealingOrchestrator (opcional).
"""
from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
import urllib.request
import urllib.error
from pathlib import Path
from typing import Callable, Optional

logger = logging.getLogger("atlas.nexus_heartbeat")

# ATLAS AUTONOMOUS: integración opcional (lazy init para no fallar si autonomous no está)
_health_aggregator = None
_metrics_aggregator = None
_healing_orchestrator = None
_circuit_breaker = None
_autonomous_initialized: bool = False
_last_nexus_success_time: float = 0


def _get_autonomous() -> bool:
    """Inicializa y devuelve True si módulos autonomous están disponibles."""
    global _health_aggregator, _metrics_aggregator, _healing_orchestrator, _circuit_breaker, _autonomous_initialized
    if _autonomous_initialized:
        return _health_aggregator is not None
    _autonomous_initialized = True
    try:
        import sys
        base = Path(__file__).resolve().parent.parent
        if str(base) not in sys.path:
            sys.path.insert(0, str(base))
        from autonomous.health_monitor.health_aggregator import HealthAggregator
        from autonomous.telemetry.metrics_aggregator import MetricsAggregator
        from autonomous.self_healing.circuit_breaker import CircuitBreaker
        from autonomous.self_healing.healing_orchestrator import HealingOrchestrator
        _health_aggregator = HealthAggregator()
        _metrics_aggregator = MetricsAggregator()
        _healing_orchestrator = HealingOrchestrator()
        _circuit_breaker = CircuitBreaker.get("nexus_heartbeat", {"failure_threshold": 3, "timeout_seconds": 30})
        logger.info("Heartbeat integrado con ATLAS AUTONOMOUS (Health, CircuitBreaker, Healing)")
        return True
    except Exception as e:
        logger.debug("Autonomous no disponible para heartbeat: %s", e)
        _health_aggregator = _metrics_aggregator = _healing_orchestrator = _circuit_breaker = None
        return False

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

# Backoff para evitar spam en Bitácora durante reconexión.
_DISCONNECT_EMIT_BACKOFF = [5, 10, 30]
_disconnect_emit_idx = 0
_disconnect_next_emit_ts: float = 0.0


def ping_nexus() -> tuple[bool, str]:
    """GET NEXUS_HEALTH_URL; si falla (p. ej. 404), intenta /status. Devuelve (ok, mensaje)."""
    base = os.getenv("NEXUS_BASE_URL", "http://127.0.0.1:8000").rstrip("/")
    for path in ("/health", "/status"):
        url = base + path
        try:
            req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=NEXUS_TIMEOUT) as r:
                if r.status == 200:
                    return (True, "OK")
                if path == "/health":
                    continue
                return (False, f"HTTP {r.status}")
        except urllib.error.HTTPError as e:
            if e.code == 404 and path == "/health":
                continue
            return (False, f"HTTP {e.code}")
        except Exception as e:
            if path == "/health":
                continue
            return (False, str(e)[:120])
    return (False, "no response from /health nor /status")


def _free_port_8000() -> None:
    """Libera el puerto 8000 (mata proceso que lo use) para evitar Errno 10048."""
    base = Path(__file__).resolve().parent.parent
    free_port = base / "scripts" / "free_port.ps1"
    if not free_port.exists():
        free_port = base / "scripts" / "free_port_8000.ps1"
    if free_port.exists():
        try:
            args = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(free_port)]
            if "free_port_8000" in str(free_port):
                args.append("-Kill")
            else:
                args.extend(["-Port", "8000", "-Kill"])
            subprocess.run(args, cwd=str(base), creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0, timeout=15, capture_output=True)
        except Exception as e:
            logger.debug("free_port al liberar 8000: %s", e)


def clear_nexus_cache() -> None:
    """Limpia __pycache__ en nexus/atlas_nexus y temp_models_cache para evitar estado viejo en navegador."""
    base = Path(__file__).resolve().parent.parent
    import shutil
    for dir_path in (base / "nexus" / "atlas_nexus", base / "modules", base / "atlas_adapter"):
        if dir_path.exists():
            for pycache in dir_path.rglob("__pycache__"):
                try:
                    shutil.rmtree(pycache, ignore_errors=True)
                except Exception:
                    pass
    temp_cache = base / "temp_models_cache"
    if temp_cache.exists():
        for p in temp_cache.iterdir():
            try:
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
                else:
                    p.unlink(missing_ok=True)
            except Exception:
                pass
    logger.info("Cache limpiado (__pycache__, temp_models_cache).")


def restart_nexus() -> bool:
    """Ejecuta NEXUS_START_SCRIPT o arranca NEXUS desde el repo (start.ps1 o python). Devuelve True si se lanzó."""
    base = Path(__file__).resolve().parent.parent  # ATLAS_PUSH
    _free_port_8000()
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
    global _nexus_connected, _nexus_last_check_ts, _nexus_last_error, _disconnect_emit_idx, _disconnect_next_emit_ts
    prev = _nexus_connected
    _nexus_connected = bool(connected)
    _nexus_last_check_ts = time.time()
    _nexus_last_error = message if not connected else ""
    # Notificación resiliente:
    # - Conectado: solo cuando cambia de False->True (sin spam).
    # - Desconectado: cuando cambia de True->False o en backoff (5s, 10s, 30s...) mientras sigue desconectado.
    notify = False
    now = time.time()
    if _nexus_connected:
        _disconnect_emit_idx = 0
        _disconnect_next_emit_ts = 0.0
        if not prev:
            notify = True
    else:
        if prev:
            _disconnect_emit_idx = 0
            _disconnect_next_emit_ts = 0.0
            notify = True
        else:
            if now >= float(_disconnect_next_emit_ts or 0.0):
                notify = True
        if notify:
            backoff = _DISCONNECT_EMIT_BACKOFF[min(_disconnect_emit_idx, len(_DISCONNECT_EMIT_BACKOFF) - 1)]
            _disconnect_next_emit_ts = now + float(backoff)
            if _disconnect_emit_idx < len(_DISCONNECT_EMIT_BACKOFF) - 1:
                _disconnect_emit_idx += 1

    if notify and _on_status_change:
        try:
            _on_status_change(_nexus_connected, message)
        except Exception:
            pass


def register_status_callback(callback: Callable[[bool, str], None]) -> None:
    """Registra callback al cambiar estado (conectado/desconectado). Para enviar a Bitácora/Dashboard."""
    global _on_status_change
    _on_status_change = callback


# Backoff (segundos) antes de cada reintento de restart (resiliencia prompt Claude NEXUS)
try:
    from modules.humanoid.resilience.retry_policy import RetryPolicy

    _RESTART_POLICY = RetryPolicy.fixed([5, 10, 30], jitter=0.0)
except Exception:
    _RESTART_POLICY = None
_restart_count = 0


def _heartbeat_loop() -> None:
    """Bucle: ping cada NEXUS_HEARTBEAT_INTERVAL_SEC; si falla, intenta restart_nexus() con backoff.
    Integración ATLAS AUTONOMOUS: métricas, circuit breaker, healing tras 3 fallos.
    """
    global _nexus_connected, _restart_count, _last_nexus_success_time
    consecutive_failures = 0
    has_autonomous = _get_autonomous()
    while not _heartbeat_stop.wait(timeout=NEXUS_HEARTBEAT_INTERVAL_SEC):
        # Circuit breaker: si está OPEN, no hacer ping y marcar desconectado
        if has_autonomous and _circuit_breaker and _circuit_breaker.get_state().value == "open":
            set_nexus_connected(False, "circuit open")
            consecutive_failures += 1
            if consecutive_failures >= 3 and _healing_orchestrator:
                try:
                    _healing_orchestrator.handle_error(
                        Exception("NEXUS circuit open"),
                        {"service": "nexus", "consecutive_failures": consecutive_failures, "last_success": _last_nexus_success_time},
                    )
                except Exception as e:
                    logger.debug("Healing handle_error: %s", e)
            continue
        t0 = time.perf_counter()
        ok, msg = ping_nexus()
        latency_ms = (time.perf_counter() - t0) * 1000
        if has_autonomous and _metrics_aggregator:
            try:
                _metrics_aggregator.collect_metric("heartbeat", "nexus_latency_ms", latency_ms)
                _metrics_aggregator.collect_metric("heartbeat", "nexus_online", 1.0 if ok else 0.0)
            except Exception as e:
                logger.debug("Métricas heartbeat: %s", e)
        if ok:
            _last_nexus_success_time = time.time()
            if has_autonomous and _circuit_breaker:
                try:
                    _circuit_breaker.record_success()
                except Exception:
                    pass
            consecutive_failures = 0
            _restart_count = 0
            if not _nexus_connected:
                set_nexus_connected(True, "")
                logger.info("[CONEXIÓN] NEXUS en puerto 8000... OK.")
        else:
            if has_autonomous and _circuit_breaker:
                try:
                    _circuit_breaker.record_failure()
                except Exception:
                    pass
            consecutive_failures += 1
            set_nexus_connected(False, msg)
            if consecutive_failures >= 3 and _healing_orchestrator:
                try:
                    _healing_orchestrator.handle_error(
                        Exception(msg),
                        {"service": "nexus", "consecutive_failures": consecutive_failures, "last_success": _last_nexus_success_time},
                    )
                except Exception as e:
                    logger.debug("Healing handle_error: %s", e)
            if consecutive_failures >= 2:
                delay = 5
                try:
                    if _RESTART_POLICY is not None:
                        delay = float(_RESTART_POLICY.delay_for_attempt(_restart_count))
                    else:
                        delay = [5, 10, 30][min(_restart_count, 2)]
                except Exception:
                    delay = 5
                logger.warning(
                    "[CONEXIÓN] NEXUS no responde. Auto-reactivación en %ss (intento %s): %s",
                    delay, _restart_count + 1, NEXUS_START_SCRIPT,
                )
                if _heartbeat_stop.wait(timeout=delay):
                    break
                restart_nexus()
                _restart_count += 1
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
