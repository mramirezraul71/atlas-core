"""
Watchdog: Vigilancia y heartbeats del tronco encefalico.

Analogo biologico: Sistema de alerta/vigilancia
- Monitoreo de heartbeats de modulos
- Deteccion de modulos colgados
- Reinicio automatico de servicios
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="brainstem")
    except Exception:
        pass


@dataclass
class ModuleStatus:
    """Estado de un modulo monitoreado."""
    module_id: str
    name: str
    last_heartbeat_ns: int = 0
    expected_interval_ms: float = 1000.0
    consecutive_misses: int = 0
    is_critical: bool = False
    auto_restart: bool = True
    status: str = "unknown"  # healthy, warning, dead, restarting
    
    def age_ms(self) -> float:
        """Edad del ultimo heartbeat en ms."""
        return (time.time_ns() - self.last_heartbeat_ns) / 1e6
    
    def is_overdue(self) -> bool:
        """Verifica si el heartbeat esta atrasado."""
        return self.age_ms() > self.expected_interval_ms * 2
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "module_id": self.module_id,
            "name": self.name,
            "status": self.status,
            "age_ms": self.age_ms(),
            "consecutive_misses": self.consecutive_misses,
            "is_critical": self.is_critical,
        }


class Watchdog:
    """
    Watchdog del tronco encefalico.
    
    Responsabilidades:
    - Recibir heartbeats de modulos
    - Detectar modulos colgados
    - Ejecutar acciones de recuperacion
    """
    
    def __init__(self, 
                 check_interval_s: float = 0.5,
                 max_misses_before_action: int = 3):
        """
        Inicializa el watchdog.
        
        Args:
            check_interval_s: Intervalo de verificacion
            max_misses_before_action: Heartbeats perdidos antes de accion
        """
        self.check_interval_s = check_interval_s
        self.max_misses_before_action = max_misses_before_action
        
        # Modulos monitoreados
        self._modules: Dict[str, ModuleStatus] = {}
        
        # Callbacks
        self._on_module_dead: List[Callable[[ModuleStatus], None]] = []
        self._on_module_recovered: List[Callable[[ModuleStatus], None]] = []
        self._restart_handlers: Dict[str, Callable[[], bool]] = {}
        
        # Estado
        self._running = False
        self._watch_task: Optional[asyncio.Task] = None
        
        # Estadisticas
        self._total_restarts = 0
        self._total_deaths_detected = 0
    
    async def start(self) -> None:
        """Inicia el watchdog."""
        if self._running:
            return
        
        self._running = True
        self._watch_task = asyncio.create_task(self._watch_loop())
        logger.info("Watchdog started")
    
    async def stop(self) -> None:
        """Detiene el watchdog."""
        self._running = False
        if self._watch_task:
            self._watch_task.cancel()
            try:
                await self._watch_task
            except asyncio.CancelledError:
                pass
        logger.info("Watchdog stopped")
    
    async def _watch_loop(self) -> None:
        """Loop principal de vigilancia."""
        while self._running:
            try:
                await self._check_all_modules()
                await asyncio.sleep(self.check_interval_s)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in watchdog loop: {e}")
                await asyncio.sleep(self.check_interval_s)
    
    async def _check_all_modules(self) -> None:
        """Verifica todos los modulos."""
        for module_id, module in self._modules.items():
            await self._check_module(module)
    
    async def _check_module(self, module: ModuleStatus) -> None:
        """Verifica un modulo individual."""
        if module.status == "restarting":
            return  # Ignorar modulos en proceso de reinicio
        
        if module.is_overdue():
            module.consecutive_misses += 1
            
            if module.consecutive_misses == 1:
                # Primera falta - advertencia
                module.status = "warning"
                logger.warning(f"Module {module.name} missed heartbeat "
                             f"(age: {module.age_ms():.0f}ms)")
            
            elif module.consecutive_misses >= self.max_misses_before_action:
                # Demasiadas faltas - modulo muerto
                module.status = "dead"
                self._total_deaths_detected += 1
                
                logger.error(f"Module {module.name} is DEAD "
                           f"(missed {module.consecutive_misses} heartbeats)")
                _bitacora(f"Watchdog: módulo {module.module_id} sin respuesta", ok=False)
                
                # Notificar
                await self._notify_module_dead(module)
                
                # Intentar reiniciar
                if module.auto_restart:
                    await self._try_restart_module(module)
        else:
            # Heartbeat recibido a tiempo
            if module.status in ("warning", "dead"):
                # Modulo se recupero
                old_status = module.status
                module.status = "healthy"
                module.consecutive_misses = 0
                
                if old_status == "dead":
                    await self._notify_module_recovered(module)
            else:
                module.status = "healthy"
                module.consecutive_misses = 0
    
    async def _try_restart_module(self, module: ModuleStatus) -> None:
        """Intenta reiniciar un modulo."""
        if module.module_id not in self._restart_handlers:
            logger.warning(f"No restart handler for module {module.name}")
            return
        
        module.status = "restarting"
        handler = self._restart_handlers[module.module_id]
        
        try:
            logger.info(f"Attempting to restart module {module.name}")
            
            if asyncio.iscoroutinefunction(handler):
                success = await handler()
            else:
                success = handler()
            
            if success:
                self._total_restarts += 1
                module.status = "healthy"
                module.consecutive_misses = 0
                module.last_heartbeat_ns = time.time_ns()
                logger.info(f"Module {module.name} restarted successfully")
            else:
                module.status = "dead"
                logger.error(f"Failed to restart module {module.name}")
                
        except Exception as e:
            module.status = "dead"
            logger.error(f"Error restarting module {module.name}: {e}")
    
    async def _notify_module_dead(self, module: ModuleStatus) -> None:
        """Notifica que un modulo murio."""
        for callback in self._on_module_dead:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(module)
                else:
                    callback(module)
            except Exception as e:
                logger.error(f"Error in module dead callback: {e}")
    
    async def _notify_module_recovered(self, module: ModuleStatus) -> None:
        """Notifica que un modulo se recupero."""
        for callback in self._on_module_recovered:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(module)
                else:
                    callback(module)
            except Exception as e:
                logger.error(f"Error in module recovered callback: {e}")
    
    def register_module(self,
                       module_id: str,
                       name: str,
                       expected_interval_ms: float = 1000.0,
                       is_critical: bool = False,
                       auto_restart: bool = True,
                       restart_handler: Callable[[], bool] = None) -> None:
        """
        Registra un modulo para monitoreo.
        
        Args:
            module_id: ID unico del modulo
            name: Nombre descriptivo
            expected_interval_ms: Intervalo esperado de heartbeat
            is_critical: Si es critico para el sistema
            auto_restart: Si intentar reinicio automatico
            restart_handler: Funcion para reiniciar el modulo
        """
        self._modules[module_id] = ModuleStatus(
            module_id=module_id,
            name=name,
            last_heartbeat_ns=time.time_ns(),
            expected_interval_ms=expected_interval_ms,
            is_critical=is_critical,
            auto_restart=auto_restart,
            status="healthy",
        )
        
        if restart_handler:
            self._restart_handlers[module_id] = restart_handler
        
        logger.info(f"Module registered for watchdog: {name} ({module_id})")
    
    def unregister_module(self, module_id: str) -> bool:
        """Desregistra un modulo."""
        if module_id in self._modules:
            del self._modules[module_id]
            if module_id in self._restart_handlers:
                del self._restart_handlers[module_id]
            return True
        return False
    
    def heartbeat(self, module_id: str) -> bool:
        """
        Recibe heartbeat de un modulo.
        
        Args:
            module_id: ID del modulo
        
        Returns:
            True si el modulo esta registrado
        """
        module = self._modules.get(module_id)
        if not module:
            return False
        
        module.last_heartbeat_ns = time.time_ns()
        
        # Recuperar si estaba en warning
        if module.status == "warning":
            module.status = "healthy"
            module.consecutive_misses = 0
        
        return True
    
    def get_module_status(self, module_id: str) -> Optional[ModuleStatus]:
        """Obtiene estado de un modulo."""
        return self._modules.get(module_id)
    
    def get_all_statuses(self) -> Dict[str, ModuleStatus]:
        """Obtiene estado de todos los modulos."""
        return self._modules.copy()
    
    def get_dead_modules(self) -> List[ModuleStatus]:
        """Obtiene lista de modulos muertos."""
        return [m for m in self._modules.values() if m.status == "dead"]
    
    def get_critical_dead_modules(self) -> List[ModuleStatus]:
        """Obtiene modulos criticos muertos."""
        return [m for m in self._modules.values() 
                if m.status == "dead" and m.is_critical]
    
    def is_system_healthy(self) -> bool:
        """Verifica si el sistema esta saludable."""
        # Sistema saludable si no hay modulos criticos muertos
        return len(self.get_critical_dead_modules()) == 0
    
    def on_module_dead(self, callback: Callable[[ModuleStatus], None]) -> None:
        """Registra callback para modulos muertos."""
        self._on_module_dead.append(callback)
    
    def on_module_recovered(self, callback: Callable[[ModuleStatus], None]) -> None:
        """Registra callback para modulos recuperados."""
        self._on_module_recovered.append(callback)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del watchdog."""
        modules_by_status = {}
        for module in self._modules.values():
            status = module.status
            modules_by_status[status] = modules_by_status.get(status, 0) + 1
        
        return {
            "running": self._running,
            "total_modules": len(self._modules),
            "modules_by_status": modules_by_status,
            "dead_modules": [m.name for m in self.get_dead_modules()],
            "critical_dead": [m.name for m in self.get_critical_dead_modules()],
            "is_system_healthy": self.is_system_healthy(),
            "total_restarts": self._total_restarts,
            "total_deaths_detected": self._total_deaths_detected,
        }
