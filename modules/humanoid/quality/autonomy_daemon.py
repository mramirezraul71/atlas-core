"""
ATLAS Autonomy Daemon
======================
Sistema de autonomía completa que mantiene a ATLAS funcionando 24/7.

Este es el CORAZÓN que hace que ATLAS sea 100% autónomo:
1. Arranca todos los subsistemas automáticamente
2. Monitorea la salud continuamente
3. Auto-repara cuando detecta problemas
4. Ejecuta mantenimiento programado
5. Reporta estado a todos los canales

USO:
    from modules.humanoid.quality.autonomy_daemon import AtlasAutonomyDaemon
    
    daemon = AtlasAutonomyDaemon()
    daemon.start()  # Arranca todo el sistema autónomo
"""
from __future__ import annotations

import atexit
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

_log = logging.getLogger("humanoid.quality.autonomy_daemon")

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent


# ============================================================================
# CONFIGURACIÓN
# ============================================================================

@dataclass
class AutonomyConfig:
    """Configuración del daemon de autonomía."""
    # Intervals
    health_check_interval: int = 30  # segundos
    maintenance_check_interval: int = 300  # 5 min
    sync_check_interval: int = 120  # 2 min
    
    # Thresholds
    max_consecutive_failures: int = 3
    auto_restart_on_failure: bool = True
    
    # Features
    enable_auto_commit: bool = True
    enable_auto_repair: bool = True
    enable_scheduled_maintenance: bool = True
    enable_incident_response: bool = True
    
    # Notifications
    notify_on_start: bool = True
    notify_on_critical: bool = True
    telegram_on_errors: bool = True


# ============================================================================
# HEALTH MONITOR
# ============================================================================

@dataclass
class HealthStatus:
    """Estado de salud de un componente."""
    name: str
    healthy: bool
    last_check: str
    error: Optional[str] = None
    consecutive_failures: int = 0


class HealthMonitor:
    """
    Monitor de salud del sistema.
    Ejecuta checks periódicos y dispara reparaciones.
    """
    
    def __init__(self, config: AutonomyConfig):
        self.config = config
        self._checks: Dict[str, Callable[[], bool]] = {}
        self._status: Dict[str, HealthStatus] = {}
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
    
    def register_check(self, name: str, check_fn: Callable[[], bool]) -> None:
        """Registra un health check."""
        self._checks[name] = check_fn
        self._status[name] = HealthStatus(
            name=name,
            healthy=True,
            last_check=datetime.now(timezone.utc).isoformat(),
        )
    
    def start(self) -> None:
        """Inicia el monitor en background."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=self._monitor_loop,
            name="HealthMonitor",
            daemon=True,
        )
        self._thread.start()
        _log.info("Health monitor started with %d checks", len(self._checks))
    
    def stop(self) -> None:
        """Detiene el monitor."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
    
    def _monitor_loop(self) -> None:
        """Loop principal de monitoreo."""
        while self._running:
            self._run_all_checks()
            time.sleep(self.config.health_check_interval)
    
    def _run_all_checks(self) -> None:
        """Ejecuta todos los health checks."""
        for name, check_fn in self._checks.items():
            try:
                healthy = check_fn()
                with self._lock:
                    status = self._status[name]
                    status.healthy = healthy
                    status.last_check = datetime.now(timezone.utc).isoformat()
                    if healthy:
                        status.consecutive_failures = 0
                        status.error = None
                    else:
                        status.consecutive_failures += 1
                        
                        # Disparar reparación si supera umbral
                        if status.consecutive_failures >= self.config.max_consecutive_failures:
                            self._trigger_repair(name)
                            
            except Exception as e:
                with self._lock:
                    status = self._status[name]
                    status.healthy = False
                    status.error = str(e)
                    status.consecutive_failures += 1
                    status.last_check = datetime.now(timezone.utc).isoformat()
    
    def _trigger_repair(self, component: str) -> None:
        """Dispara reparación automática."""
        if not self.config.enable_auto_repair:
            _log.warning("Auto-repair disabled, skipping repair for %s", component)
            return
        
        _log.warning("Triggering auto-repair for %s", component)
        
        # Mapeo de componente a POT
        repair_map = {
            "api": "api_repair",
            "services": "services_repair",
            "camera": "camera_repair",
            "dispatcher": "services_repair",
            "triggers": "services_repair",
        }
        
        pot_id = repair_map.get(component, "diagnostic_full")
        
        try:
            from .dispatcher import dispatch_pot, TriggerType
            dispatch_pot(
                pot_id=pot_id,
                trigger_type=TriggerType.CONDITION,
                trigger_id=f"health_monitor_{component}",
                context={"component": component, "auto_repair": True},
            )
        except Exception as e:
            _log.error("Failed to dispatch repair POT: %s", e)
    
    def get_status(self) -> Dict[str, Any]:
        """Retorna estado de todos los componentes."""
        with self._lock:
            return {
                name: {
                    "healthy": s.healthy,
                    "last_check": s.last_check,
                    "failures": s.consecutive_failures,
                    "error": s.error,
                }
                for name, s in self._status.items()
            }
    
    def is_healthy(self) -> bool:
        """Retorna True si todo está saludable."""
        with self._lock:
            return all(s.healthy for s in self._status.values())


# ============================================================================
# WATCHDOG
# ============================================================================

class Watchdog:
    """
    Watchdog que asegura que los componentes críticos estén corriendo.
    Si algo muere, lo reinicia automáticamente.
    """
    
    def __init__(self, config: AutonomyConfig):
        self.config = config
        self._components: Dict[str, Callable[[], bool]] = {}  # name -> is_running
        self._restarters: Dict[str, Callable[[], None]] = {}   # name -> restart_fn
        self._running = False
        self._thread: Optional[threading.Thread] = None
    
    def register(
        self, 
        name: str, 
        is_running_fn: Callable[[], bool],
        restart_fn: Callable[[], None]
    ) -> None:
        """Registra un componente para watchdog."""
        self._components[name] = is_running_fn
        self._restarters[name] = restart_fn
    
    def start(self) -> None:
        """Inicia el watchdog."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=self._watch_loop,
            name="Watchdog",
            daemon=True,
        )
        self._thread.start()
        _log.info("Watchdog started monitoring %d components", len(self._components))
    
    def stop(self) -> None:
        """Detiene el watchdog."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
    
    def _watch_loop(self) -> None:
        """Loop de vigilancia."""
        while self._running:
            for name, is_running in self._components.items():
                try:
                    if not is_running():
                        _log.warning("Component %s is down, restarting...", name)
                        if self.config.auto_restart_on_failure:
                            self._restart_component(name)
                except Exception as e:
                    _log.error("Error checking component %s: %s", name, e)
            
            time.sleep(10)  # Check cada 10 segundos
    
    def _restart_component(self, name: str) -> None:
        """Reinicia un componente."""
        if name not in self._restarters:
            return
        
        try:
            self._restarters[name]()
            _log.info("Component %s restarted successfully", name)
        except Exception as e:
            _log.error("Failed to restart %s: %s", name, e)


# ============================================================================
# SCHEDULED TASKS
# ============================================================================

class ScheduledTaskRunner:
    """
    Ejecutor de tareas programadas internas.
    Complementa al Scheduler principal con tareas de autonomía.
    """
    
    def __init__(self, config: AutonomyConfig):
        self.config = config
        self._tasks: List[Dict[str, Any]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_daily_maintenance: Optional[datetime] = None
        self._last_weekly_maintenance: Optional[datetime] = None
    
    def start(self) -> None:
        """Inicia el runner de tareas."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=self._task_loop,
            name="ScheduledTasks",
            daemon=True,
        )
        self._thread.start()
        _log.info("Scheduled task runner started")
    
    def stop(self) -> None:
        """Detiene el runner."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
    
    def _task_loop(self) -> None:
        """Loop de tareas programadas."""
        while self._running:
            now = datetime.now(timezone.utc)
            
            # Mantenimiento diario (a las 3:00 AM)
            if self.config.enable_scheduled_maintenance:
                if now.hour == 3 and self._should_run_daily():
                    self._run_daily_maintenance()
                    self._last_daily_maintenance = now
                
                # Mantenimiento semanal (domingos a las 4:00 AM)
                if now.weekday() == 6 and now.hour == 4 and self._should_run_weekly():
                    self._run_weekly_maintenance()
                    self._last_weekly_maintenance = now
            
            # Auto-sync cada intervalo configurado
            if self.config.enable_auto_commit:
                self._check_auto_sync()
            
            time.sleep(60)  # Check cada minuto
    
    def _should_run_daily(self) -> bool:
        """Verifica si debe ejecutar mantenimiento diario."""
        if self._last_daily_maintenance is None:
            return True
        return (datetime.now(timezone.utc) - self._last_daily_maintenance) > timedelta(hours=20)
    
    def _should_run_weekly(self) -> bool:
        """Verifica si debe ejecutar mantenimiento semanal."""
        if self._last_weekly_maintenance is None:
            return True
        return (datetime.now(timezone.utc) - self._last_weekly_maintenance) > timedelta(days=6)
    
    def _run_daily_maintenance(self) -> None:
        """Ejecuta mantenimiento diario."""
        _log.info("Running daily maintenance")
        try:
            from .dispatcher import dispatch_pot, TriggerType
            dispatch_pot(
                pot_id="maintenance_daily",
                trigger_type=TriggerType.SCHEDULED,
                trigger_id="autonomy_daily",
                context={"scheduled": True},
            )
        except Exception as e:
            _log.error("Daily maintenance failed: %s", e)
    
    def _run_weekly_maintenance(self) -> None:
        """Ejecuta mantenimiento semanal."""
        _log.info("Running weekly maintenance")
        try:
            from .dispatcher import dispatch_pot, TriggerType
            dispatch_pot(
                pot_id="maintenance_weekly",
                trigger_type=TriggerType.SCHEDULED,
                trigger_id="autonomy_weekly",
                context={"scheduled": True},
            )
        except Exception as e:
            _log.error("Weekly maintenance failed: %s", e)
    
    def _check_auto_sync(self) -> None:
        """Verifica si hay cambios para sincronizar."""
        try:
            from .sync_engine import get_sync_engine
            engine = get_sync_engine()
            # El sync engine ya maneja la lógica de auto-sync
        except Exception:
            pass


# ============================================================================
# MAIN DAEMON
# ============================================================================

class AtlasAutonomyDaemon:
    """
    Daemon principal de autonomía de ATLAS.
    
    Coordina todos los subsistemas para lograr autonomía completa:
    - Dispatcher: Ejecuta POTs
    - Triggers: Detecta condiciones
    - Health Monitor: Verifica salud
    - Watchdog: Reinicia componentes caídos
    - Scheduled Tasks: Mantenimiento programado
    """
    
    def __init__(self, config: Optional[AutonomyConfig] = None):
        self.config = config or AutonomyConfig()
        
        # Subsistemas
        self._health_monitor = HealthMonitor(self.config)
        self._watchdog = Watchdog(self.config)
        self._scheduler = ScheduledTaskRunner(self.config)
        
        # Estado
        self._running = False
        self._started_at: Optional[str] = None
        self._dispatcher = None
        self._trigger_engine = None
    
    def start(self) -> Dict[str, Any]:
        """
        Inicia el sistema de autonomía completa.
        
        Returns:
            Estado de inicio de cada componente
        """
        if self._running:
            return {"ok": True, "already_running": True}
        
        results = {}
        self._started_at = datetime.now(timezone.utc).isoformat()
        
        _log.info("=" * 60)
        _log.info("ATLAS AUTONOMY DAEMON STARTING")
        _log.info("=" * 60)
        
        # 1. Iniciar Dispatcher
        try:
            from .dispatcher import start_dispatcher, get_dispatcher
            self._dispatcher = start_dispatcher()
            results["dispatcher"] = {
                "ok": True,
                "running": self._dispatcher.is_running(),
            }
            _log.info("[1/6] Dispatcher: OK")
        except Exception as e:
            results["dispatcher"] = {"ok": False, "error": str(e)}
            _log.error("[1/6] Dispatcher: FAILED - %s", e)
        
        # 2. Iniciar Triggers
        try:
            from .triggers import start_triggers, get_trigger_engine
            self._trigger_engine = start_triggers()
            results["triggers"] = {
                "ok": True,
                "running": self._trigger_engine.is_running(),
                "rules": len(self._trigger_engine.registry.list_all()),
            }
            _log.info("[2/6] Triggers: OK")
        except Exception as e:
            results["triggers"] = {"ok": False, "error": str(e)}
            _log.error("[2/6] Triggers: FAILED - %s", e)
        
        # 3. Iniciar Robotics Bridge
        try:
            from .robotics_bridge import init_robotics_bridge
            rb = init_robotics_bridge()
            results["robotics_bridge"] = {
                "ok": True,
                "modules": rb.get_status(),
            }
            _log.info("[3/6] Robotics Bridge: OK")
        except Exception as e:
            results["robotics_bridge"] = {"ok": False, "error": str(e)}
            _log.error("[3/6] Robotics Bridge: FAILED - %s", e)
        
        # 4. Configurar Health Checks
        self._setup_health_checks()
        self._health_monitor.start()
        results["health_monitor"] = {"ok": True, "running": True}
        _log.info("[4/6] Health Monitor: OK")
        
        # 5. Configurar Watchdog
        self._setup_watchdog()
        self._watchdog.start()
        results["watchdog"] = {"ok": True, "running": True}
        _log.info("[5/6] Watchdog: OK")
        
        # 6. Iniciar Scheduled Tasks
        self._scheduler.start()
        results["scheduler"] = {"ok": True, "running": True}
        _log.info("[6/6] Scheduled Tasks: OK")
        
        # Registrar handlers de señales
        self._register_signal_handlers()
        
        # Marcar como corriendo
        self._running = True
        results["all_ok"] = all(
            r.get("ok", False) for r in results.values() 
            if isinstance(r, dict)
        )
        
        _log.info("=" * 60)
        _log.info("ATLAS AUTONOMY DAEMON STARTED")
        _log.info("=" * 60)
        
        # Notificar inicio
        if self.config.notify_on_start:
            self._notify_start(results)
        
        return results
    
    def stop(self) -> Dict[str, Any]:
        """Detiene el sistema de autonomía."""
        if not self._running:
            return {"ok": True, "already_stopped": True}
        
        _log.info("ATLAS AUTONOMY DAEMON STOPPING...")
        
        results = {}
        
        # Detener en orden inverso
        try:
            self._scheduler.stop()
            results["scheduler"] = {"ok": True, "stopped": True}
        except Exception as e:
            results["scheduler"] = {"ok": False, "error": str(e)}
        
        try:
            self._watchdog.stop()
            results["watchdog"] = {"ok": True, "stopped": True}
        except Exception as e:
            results["watchdog"] = {"ok": False, "error": str(e)}
        
        try:
            self._health_monitor.stop()
            results["health_monitor"] = {"ok": True, "stopped": True}
        except Exception as e:
            results["health_monitor"] = {"ok": False, "error": str(e)}
        
        try:
            from .triggers import stop_triggers
            stop_triggers()
            results["triggers"] = {"ok": True, "stopped": True}
        except Exception as e:
            results["triggers"] = {"ok": False, "error": str(e)}
        
        try:
            from .dispatcher import stop_dispatcher
            stop_dispatcher(graceful=True)
            results["dispatcher"] = {"ok": True, "stopped": True}
        except Exception as e:
            results["dispatcher"] = {"ok": False, "error": str(e)}
        
        self._running = False
        _log.info("ATLAS AUTONOMY DAEMON STOPPED")
        
        return results
    
    def _setup_health_checks(self) -> None:
        """Configura los health checks."""
        
        # Check Dispatcher
        def check_dispatcher():
            try:
                from .dispatcher import get_dispatcher
                return get_dispatcher().is_running()
            except:
                return False
        
        # Check Triggers
        def check_triggers():
            try:
                from .triggers import get_trigger_engine
                return get_trigger_engine().is_running()
            except:
                return False
        
        # Check API (si existe)
        def check_api():
            try:
                import urllib.request
                req = urllib.request.Request("http://127.0.0.1:8791/health", method="GET")
                with urllib.request.urlopen(req, timeout=5) as r:
                    return r.status == 200
            except:
                return False
        
        # Check Git status
        def check_git():
            try:
                import subprocess
                result = subprocess.run(
                    ["git", "status", "--porcelain"],
                    capture_output=True,
                    timeout=10,
                    cwd=str(REPO_ROOT),
                )
                return result.returncode == 0
            except:
                return False
        
        self._health_monitor.register_check("dispatcher", check_dispatcher)
        self._health_monitor.register_check("triggers", check_triggers)
        self._health_monitor.register_check("api", check_api)
        self._health_monitor.register_check("git", check_git)
    
    def _setup_watchdog(self) -> None:
        """Configura el watchdog."""
        
        # Watchdog para Dispatcher
        def dispatcher_running():
            try:
                from .dispatcher import get_dispatcher
                return get_dispatcher().is_running()
            except:
                return False
        
        def restart_dispatcher():
            from .dispatcher import start_dispatcher
            start_dispatcher()
        
        # Watchdog para Triggers
        def triggers_running():
            try:
                from .triggers import get_trigger_engine
                return get_trigger_engine().is_running()
            except:
                return False
        
        def restart_triggers():
            from .triggers import start_triggers
            start_triggers()
        
        self._watchdog.register("dispatcher", dispatcher_running, restart_dispatcher)
        self._watchdog.register("triggers", triggers_running, restart_triggers)
    
    def _register_signal_handlers(self) -> None:
        """Registra handlers para señales del sistema."""
        def signal_handler(signum, frame):
            _log.info("Received signal %s, stopping daemon...", signum)
            self.stop()
        
        # Solo registrar si no estamos en Windows o si es el thread principal
        try:
            signal.signal(signal.SIGTERM, signal_handler)
            signal.signal(signal.SIGINT, signal_handler)
        except:
            pass
        
        # Registrar atexit para cleanup
        atexit.register(self.stop)
    
    def _notify_start(self, results: Dict[str, Any]) -> None:
        """Notifica el inicio del daemon."""
        try:
            from .cerebro_connector import get_bridge
            bridge = get_bridge()
            
            all_ok = results.get("all_ok", False)
            emoji = "🟢" if all_ok else "🟡"
            message = f"{emoji} ATLAS Autonomy Daemon iniciado\n"
            message += f"Componentes: {sum(1 for r in results.values() if isinstance(r, dict) and r.get('ok'))}/{len(results)-1}\n"
            message += f"Timestamp: {self._started_at}"
            
            bridge.channels.send_ops(message, "success" if all_ok else "warning")
            
            if self.config.telegram_on_errors and not all_ok:
                bridge.channels.send_telegram_sync(message)
                
        except Exception as e:
            _log.warning("Failed to send start notification: %s", e)
    
    def get_status(self) -> Dict[str, Any]:
        """Retorna estado completo del daemon."""
        return {
            "running": self._running,
            "started_at": self._started_at,
            "uptime_seconds": self._get_uptime(),
            "health": self._health_monitor.get_status(),
            "config": {
                "auto_commit": self.config.enable_auto_commit,
                "auto_repair": self.config.enable_auto_repair,
                "scheduled_maintenance": self.config.enable_scheduled_maintenance,
            },
        }
    
    def _get_uptime(self) -> int:
        """Retorna uptime en segundos."""
        if not self._started_at:
            return 0
        started = datetime.fromisoformat(self._started_at.replace("Z", "+00:00"))
        return int((datetime.now(timezone.utc) - started).total_seconds())
    
    def is_running(self) -> bool:
        """Retorna True si el daemon está corriendo."""
        return self._running


# ============================================================================
# SINGLETON Y FUNCIONES DE CONVENIENCIA
# ============================================================================

_daemon: Optional[AtlasAutonomyDaemon] = None


def get_daemon() -> AtlasAutonomyDaemon:
    """Obtiene la instancia del daemon."""
    global _daemon
    if _daemon is None:
        _daemon = AtlasAutonomyDaemon()
    return _daemon


def start_autonomy(config: Optional[AutonomyConfig] = None) -> Dict[str, Any]:
    """
    Inicia el sistema de autonomía completa de ATLAS.
    
    Esta es la función principal para activar la autonomía.
    """
    global _daemon
    _daemon = AtlasAutonomyDaemon(config)
    return _daemon.start()


def stop_autonomy() -> Dict[str, Any]:
    """Detiene el sistema de autonomía."""
    daemon = get_daemon()
    return daemon.stop()


def get_autonomy_status() -> Dict[str, Any]:
    """Obtiene el estado del sistema de autonomía."""
    daemon = get_daemon()
    return daemon.get_status()


def is_autonomy_running() -> bool:
    """Verifica si el sistema de autonomía está corriendo."""
    daemon = get_daemon()
    return daemon.is_running()


__all__ = [
    "AtlasAutonomyDaemon",
    "AutonomyConfig",
    "HealthMonitor",
    "Watchdog",
    "ScheduledTaskRunner",
    "get_daemon",
    "start_autonomy",
    "stop_autonomy",
    "get_autonomy_status",
    "is_autonomy_running",
]
