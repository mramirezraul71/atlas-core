"""
GlobalState: Estados globales del tronco encefalico.

Analogo biologico: Formacion reticular
- Estados globales del sistema (on/off/safe/emergency)
- Transiciones de estado
- Coordinacion de modos
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="brainstem")
    except Exception:
        pass


class SystemMode(Enum):
    """Modos de operacion del sistema."""
    INITIALIZING = "initializing"
    NORMAL = "normal"
    SAFE = "safe"
    LEARNING = "learning"
    MAINTENANCE = "maintenance"
    EMERGENCY = "emergency"
    SHUTDOWN = "shutdown"
    SLEEP = "sleep"


@dataclass
class StateTransition:
    """Registro de transicion de estado."""
    from_mode: SystemMode
    to_mode: SystemMode
    timestamp_ns: int
    reason: str
    triggered_by: str  # "system", "user", "safety", "watchdog"


class GlobalState:
    """
    Gestor de estados globales del tronco encefalico.
    
    Responsabilidades:
    - Mantener estado global del sistema
    - Gestionar transiciones de modo
    - Coordinar componentes segun modo
    """
    
    # Transiciones permitidas
    ALLOWED_TRANSITIONS = {
        SystemMode.INITIALIZING: {SystemMode.NORMAL, SystemMode.SAFE, SystemMode.EMERGENCY, SystemMode.SHUTDOWN},
        SystemMode.NORMAL: {SystemMode.SAFE, SystemMode.LEARNING, SystemMode.MAINTENANCE, SystemMode.EMERGENCY, SystemMode.SLEEP, SystemMode.SHUTDOWN},
        SystemMode.SAFE: {SystemMode.NORMAL, SystemMode.EMERGENCY, SystemMode.SHUTDOWN},
        SystemMode.LEARNING: {SystemMode.NORMAL, SystemMode.SAFE, SystemMode.EMERGENCY},
        SystemMode.MAINTENANCE: {SystemMode.NORMAL, SystemMode.SAFE, SystemMode.EMERGENCY, SystemMode.SHUTDOWN},
        SystemMode.EMERGENCY: {SystemMode.SAFE, SystemMode.SHUTDOWN},  # Solo puede ir a SAFE despues de reset
        SystemMode.SLEEP: {SystemMode.NORMAL, SystemMode.EMERGENCY},
        SystemMode.SHUTDOWN: set(),  # No puede transicionar desde shutdown
    }
    
    def __init__(self):
        # Estado actual
        self._mode = SystemMode.INITIALIZING
        self._mode_since_ns = time.time_ns()
        
        # Historial de transiciones
        self._transitions: List[StateTransition] = []
        
        # Contexto del estado
        self._state_context: Dict[str, Any] = {}
        
        # Callbacks por modo
        self._on_enter_mode: Dict[SystemMode, List[Callable[[SystemMode], None]]] = {
            mode: [] for mode in SystemMode
        }
        self._on_exit_mode: Dict[SystemMode, List[Callable[[SystemMode], None]]] = {
            mode: [] for mode in SystemMode
        }
        self._on_any_transition: List[Callable[[SystemMode, SystemMode], None]] = []
        
        # Bloqueo de transiciones (para operaciones criticas)
        self._transition_lock = asyncio.Lock()
        self._transition_blocked = False
        self._block_reason: Optional[str] = None
    
    @property
    def mode(self) -> SystemMode:
        """Obtiene modo actual."""
        return self._mode
    
    @property
    def mode_duration_s(self) -> float:
        """Obtiene duracion en modo actual (segundos)."""
        return (time.time_ns() - self._mode_since_ns) / 1e9
    
    def is_operational(self) -> bool:
        """Verifica si el sistema esta operacional."""
        return self._mode in (SystemMode.NORMAL, SystemMode.LEARNING)
    
    def is_safe_mode(self) -> bool:
        """Verifica si esta en modo seguro."""
        return self._mode in (SystemMode.SAFE, SystemMode.EMERGENCY)
    
    def can_transition_to(self, target_mode: SystemMode) -> bool:
        """Verifica si puede transicionar a un modo."""
        if self._transition_blocked:
            return False
        return target_mode in self.ALLOWED_TRANSITIONS.get(self._mode, set())
    
    async def set_mode(self, 
                      target_mode: SystemMode,
                      reason: str = "",
                      triggered_by: str = "system",
                      force: bool = False) -> bool:
        """
        Cambia el modo del sistema.
        
        Args:
            target_mode: Modo objetivo
            reason: Razon del cambio
            triggered_by: Quien disparo el cambio
            force: Forzar cambio aunque no este permitido
        
        Returns:
            True si se cambio exitosamente
        """
        async with self._transition_lock:
            # Verificar si la transicion esta bloqueada
            if self._transition_blocked and not force:
                logger.warning(f"Mode transition blocked: {self._block_reason}")
                return False
            
            # Verificar si la transicion esta permitida
            if not force and not self.can_transition_to(target_mode):
                logger.warning(f"Transition not allowed: {self._mode} -> {target_mode}")
                return False
            
            old_mode = self._mode
            
            # Ejecutar callbacks de salida
            await self._execute_exit_callbacks(old_mode, target_mode)
            
            # Cambiar modo
            self._mode = target_mode
            self._mode_since_ns = time.time_ns()
            
            # Registrar transicion
            transition = StateTransition(
                from_mode=old_mode,
                to_mode=target_mode,
                timestamp_ns=self._mode_since_ns,
                reason=reason,
                triggered_by=triggered_by,
            )
            self._transitions.append(transition)
            
            logger.info(f"Mode changed: {old_mode.value} -> {target_mode.value} ({reason})")
            _bitacora(f"Modo global: {old_mode.value} → {target_mode.value}")
            
            # Ejecutar callbacks de entrada
            await self._execute_enter_callbacks(target_mode, old_mode)
            
            # Ejecutar callbacks generales
            for callback in self._on_any_transition:
                try:
                    if asyncio.iscoroutinefunction(callback):
                        await callback(old_mode, target_mode)
                    else:
                        callback(old_mode, target_mode)
                except Exception as e:
                    logger.error(f"Error in transition callback: {e}")
            
            return True
    
    async def _execute_exit_callbacks(self, old_mode: SystemMode, 
                                     new_mode: SystemMode) -> None:
        """Ejecuta callbacks de salida de modo."""
        for callback in self._on_exit_mode.get(old_mode, []):
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(new_mode)
                else:
                    callback(new_mode)
            except Exception as e:
                logger.error(f"Error in exit callback for {old_mode}: {e}")
    
    async def _execute_enter_callbacks(self, new_mode: SystemMode,
                                      old_mode: SystemMode) -> None:
        """Ejecuta callbacks de entrada a modo."""
        for callback in self._on_enter_mode.get(new_mode, []):
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(old_mode)
                else:
                    callback(old_mode)
            except Exception as e:
                logger.error(f"Error in enter callback for {new_mode}: {e}")
    
    async def emergency(self, reason: str = "Unknown") -> bool:
        """
        Activa modo de emergencia.
        
        Args:
            reason: Razon de la emergencia
        
        Returns:
            True si se activo exitosamente
        """
        return await self.set_mode(
            SystemMode.EMERGENCY,
            reason=reason,
            triggered_by="safety",
            force=True,  # Emergency siempre se puede activar
        )
    
    async def recover_from_emergency(self) -> bool:
        """
        Intenta recuperar de emergencia a modo seguro.
        
        Returns:
            True si se recupero exitosamente
        """
        if self._mode != SystemMode.EMERGENCY:
            return False
        
        return await self.set_mode(
            SystemMode.SAFE,
            reason="Recovery from emergency",
            triggered_by="user",
        )
    
    async def initialize_complete(self) -> bool:
        """Marca inicializacion como completa."""
        if self._mode != SystemMode.INITIALIZING:
            return False
        
        return await self.set_mode(
            SystemMode.NORMAL,
            reason="Initialization complete",
            triggered_by="system",
        )
    
    async def shutdown(self, reason: str = "Normal shutdown") -> bool:
        """Inicia apagado del sistema."""
        return await self.set_mode(
            SystemMode.SHUTDOWN,
            reason=reason,
            triggered_by="system",
            force=True,
        )
    
    def block_transitions(self, reason: str) -> None:
        """
        Bloquea transiciones de estado.
        
        Args:
            reason: Razon del bloqueo
        """
        self._transition_blocked = True
        self._block_reason = reason
        logger.info(f"State transitions blocked: {reason}")
    
    def unblock_transitions(self) -> None:
        """Desbloquea transiciones de estado."""
        self._transition_blocked = False
        self._block_reason = None
        logger.info("State transitions unblocked")
    
    def set_context(self, key: str, value: Any) -> None:
        """Establece valor en contexto de estado."""
        self._state_context[key] = value
    
    def get_context(self, key: str, default: Any = None) -> Any:
        """Obtiene valor del contexto de estado."""
        return self._state_context.get(key, default)
    
    def on_enter_mode(self, mode: SystemMode, 
                     callback: Callable[[SystemMode], None]) -> None:
        """Registra callback para entrada a modo."""
        self._on_enter_mode[mode].append(callback)
    
    def on_exit_mode(self, mode: SystemMode,
                    callback: Callable[[SystemMode], None]) -> None:
        """Registra callback para salida de modo."""
        self._on_exit_mode[mode].append(callback)
    
    def on_transition(self, 
                     callback: Callable[[SystemMode, SystemMode], None]) -> None:
        """Registra callback para cualquier transicion."""
        self._on_any_transition.append(callback)
    
    def get_recent_transitions(self, limit: int = 10) -> List[StateTransition]:
        """Obtiene transiciones recientes."""
        return self._transitions[-limit:]
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del estado."""
        return {
            "current_mode": self._mode.value,
            "mode_since_ns": self._mode_since_ns,
            "mode_duration_s": self.mode_duration_s,
            "is_operational": self.is_operational(),
            "is_safe_mode": self.is_safe_mode(),
            "transition_blocked": self._transition_blocked,
            "block_reason": self._block_reason,
            "total_transitions": len(self._transitions),
            "context": self._state_context,
        }
