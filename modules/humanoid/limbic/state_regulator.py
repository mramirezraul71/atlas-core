"""
StateRegulator: Regulador de estado interno del sistema limbico.

Analogo biologico: Hipotalamo + sistema nervioso autonomo
- Regulacion del estado interno
- Homeostasis
- Estados emocionales/funcionales
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


class SystemMode(Enum):
    """Modos de operacion del sistema."""
    NORMAL = "normal"           # Operacion normal
    SAFETY = "safety"           # Enfoque en seguridad
    LEARNING = "learning"       # Modo aprendizaje
    EFFICIENT = "efficient"     # Enfoque en eficiencia
    RECOVERY = "recovery"       # Recuperacion de error
    EMERGENCY = "emergency"     # Emergencia
    SLEEP = "sleep"             # Bajo consumo


@dataclass
class InternalState:
    """Estado interno del robot."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    # Energia y recursos
    energy_level: float = 1.0       # 0-1, relacionado con bateria
    focus_level: float = 0.8        # 0-1, capacidad de atencion
    stress_level: float = 0.2       # 0-1, nivel de estres/carga
    
    # Estados funcionales
    mode: SystemMode = SystemMode.NORMAL
    alertness: float = 0.7          # 0-1, nivel de alerta
    confidence: float = 0.8         # 0-1, confianza en acciones
    
    # Indicadores de salud
    system_health: float = 1.0      # 0-1, salud general del sistema
    motor_health: float = 1.0       # 0-1, salud de motores
    sensor_health: float = 1.0      # 0-1, salud de sensores
    
    # Contexto social
    human_present: bool = False
    interaction_active: bool = False
    last_interaction_ns: int = 0
    
    # Flags
    needs_attention: bool = False
    needs_maintenance: bool = False
    
    def is_healthy(self) -> bool:
        """Verifica si el sistema esta saludable."""
        return (
            self.system_health > 0.7 and
            self.motor_health > 0.7 and
            self.sensor_health > 0.7 and
            self.stress_level < 0.8
        )
    
    def can_operate(self) -> bool:
        """Verifica si puede operar normalmente."""
        return (
            self.mode != SystemMode.EMERGENCY and
            self.mode != SystemMode.SLEEP and
            self.energy_level > 0.1 and
            self.system_health > 0.3
        )
    
    def should_rest(self) -> bool:
        """Verifica si deberia entrar en modo descanso."""
        return (
            self.energy_level < 0.15 or
            self.stress_level > 0.9 or
            (not self.human_present and self.energy_level < 0.3)
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "energy_level": self.energy_level,
            "focus_level": self.focus_level,
            "stress_level": self.stress_level,
            "mode": self.mode.name,
            "alertness": self.alertness,
            "confidence": self.confidence,
            "system_health": self.system_health,
            "is_healthy": self.is_healthy(),
            "can_operate": self.can_operate(),
            "human_present": self.human_present,
            "interaction_active": self.interaction_active,
        }


class StateRegulator:
    """
    Regulador de estado interno del sistema limbico.
    
    Responsabilidades:
    - Mantener homeostasis del sistema
    - Regular estados funcionales
    - Adaptar comportamiento segun estado
    - Detectar y responder a cambios
    """
    
    def __init__(self, update_interval_s: float = 1.0):
        """
        Inicializa el regulador.
        
        Args:
            update_interval_s: Intervalo de actualizacion en segundos
        """
        self.update_interval_s = update_interval_s
        
        # Estado actual
        self._state = InternalState()
        
        # Historial de estados
        self._state_history: List[InternalState] = []
        self._max_history = 1000
        
        # Callbacks
        self._on_mode_change: List[Callable[[SystemMode, SystemMode], None]] = []
        self._on_alert: List[Callable[[str, float], None]] = []
        
        # Umbrales de regulacion
        self.thresholds = {
            "energy_low": 0.2,
            "energy_critical": 0.1,
            "stress_high": 0.7,
            "stress_critical": 0.9,
            "health_warning": 0.5,
            "health_critical": 0.3,
        }
        
        # Factores de decaimiento
        self.decay_rates = {
            "focus": 0.001,      # Decae lentamente
            "alertness": 0.002,  # Decae un poco mas rapido
            "stress": -0.005,    # Se recupera lentamente
        }
    
    def update(self, 
              battery_percent: float = None,
              cpu_load: float = None,
              motor_temps: Dict[str, float] = None,
              sensor_status: Dict[str, bool] = None,
              human_detected: bool = None,
              task_complexity: float = None) -> InternalState:
        """
        Actualiza el estado interno basado en inputs.
        
        Args:
            battery_percent: Nivel de bateria (0-100)
            cpu_load: Carga de CPU (0-1)
            motor_temps: Temperaturas de motores
            sensor_status: Estado de sensores
            human_detected: Si hay humano presente
            task_complexity: Complejidad de tarea actual (0-1)
        
        Returns:
            Estado actualizado
        """
        # Actualizar timestamp
        self._state.timestamp_ns = time.time_ns()
        
        # Energia basada en bateria
        if battery_percent is not None:
            self._state.energy_level = battery_percent / 100.0
        
        # Estres basado en carga y complejidad
        if cpu_load is not None or task_complexity is not None:
            stress_input = (cpu_load or 0) * 0.4 + (task_complexity or 0) * 0.6
            # Suavizar cambios de estres
            self._state.stress_level = (
                0.9 * self._state.stress_level + 
                0.1 * stress_input
            )
        
        # Salud de motores
        if motor_temps:
            max_temp = max(motor_temps.values()) if motor_temps else 25
            # Normalizar: 25C=1.0, 70C=0.0
            motor_health = max(0, 1.0 - (max_temp - 25) / 45)
            self._state.motor_health = motor_health
        
        # Salud de sensores
        if sensor_status:
            working = sum(1 for s in sensor_status.values() if s)
            total = len(sensor_status)
            self._state.sensor_health = working / total if total > 0 else 1.0
        
        # Salud general del sistema
        self._state.system_health = (
            self._state.motor_health * 0.4 +
            self._state.sensor_health * 0.4 +
            (1.0 - self._state.stress_level) * 0.2
        )
        
        # Presencia humana
        if human_detected is not None:
            was_present = self._state.human_present
            self._state.human_present = human_detected
            
            if human_detected and not was_present:
                # Humano aparecio: aumentar alerta
                self._state.alertness = min(1.0, self._state.alertness + 0.3)
                self._state.focus_level = min(1.0, self._state.focus_level + 0.2)
                self._state.last_interaction_ns = time.time_ns()
        
        # Aplicar decaimiento natural
        self._apply_decay()
        
        # Verificar y ajustar modo
        self._regulate_mode()
        
        # Verificar alertas
        self._check_alerts()
        
        # Guardar en historial
        self._record_state()
        
        return self._state
    
    def _apply_decay(self) -> None:
        """Aplica decaimiento natural a estados."""
        for attr, rate in self.decay_rates.items():
            if hasattr(self._state, attr):
                current = getattr(self._state, attr)
                new_value = max(0.0, min(1.0, current + rate))
                setattr(self._state, attr, new_value)
    
    def _regulate_mode(self) -> None:
        """Regula el modo de operacion segun estado."""
        old_mode = self._state.mode
        new_mode = old_mode
        
        # Emergencia: siempre tiene prioridad
        if self._state.system_health < self.thresholds["health_critical"]:
            new_mode = SystemMode.EMERGENCY
        elif self._state.energy_level < self.thresholds["energy_critical"]:
            new_mode = SystemMode.EMERGENCY
        
        # Recuperacion
        elif self._state.system_health < self.thresholds["health_warning"]:
            new_mode = SystemMode.RECOVERY
        
        # Descanso
        elif self._state.should_rest():
            new_mode = SystemMode.SLEEP
        
        # Seguridad si hay estres alto o humano cerca
        elif (self._state.stress_level > self.thresholds["stress_high"] or 
              (self._state.human_present and self._state.mode != SystemMode.LEARNING)):
            new_mode = SystemMode.SAFETY
        
        # Normal
        elif old_mode in (SystemMode.RECOVERY, SystemMode.SLEEP) and self._state.is_healthy():
            new_mode = SystemMode.NORMAL
        
        # Aplicar cambio de modo
        if new_mode != old_mode:
            self._change_mode(old_mode, new_mode)
    
    def _change_mode(self, old_mode: SystemMode, new_mode: SystemMode) -> None:
        """Cambia el modo y notifica."""
        self._state.mode = new_mode
        logger.info(f"Mode changed: {old_mode.name} -> {new_mode.name}")
        
        # Ajustar parametros segun modo
        if new_mode == SystemMode.SAFETY:
            self._state.alertness = min(1.0, self._state.alertness + 0.2)
        elif new_mode == SystemMode.SLEEP:
            self._state.alertness = max(0.2, self._state.alertness - 0.3)
        elif new_mode == SystemMode.LEARNING:
            self._state.focus_level = min(1.0, self._state.focus_level + 0.2)
        
        # Notificar callbacks
        for callback in self._on_mode_change:
            try:
                callback(old_mode, new_mode)
            except Exception as e:
                logger.error(f"Error in mode change callback: {e}")
    
    def _check_alerts(self) -> None:
        """Verifica condiciones de alerta."""
        alerts = []
        
        if self._state.energy_level < self.thresholds["energy_low"]:
            alerts.append(("low_energy", self._state.energy_level))
        
        if self._state.stress_level > self.thresholds["stress_high"]:
            alerts.append(("high_stress", self._state.stress_level))
        
        if self._state.system_health < self.thresholds["health_warning"]:
            alerts.append(("low_health", self._state.system_health))
        
        # Emitir alertas
        for alert_type, value in alerts:
            self._state.needs_attention = True
            for callback in self._on_alert:
                try:
                    callback(alert_type, value)
                except Exception as e:
                    logger.error(f"Error in alert callback: {e}")
    
    def _record_state(self) -> None:
        """Guarda estado en historial."""
        # Crear copia del estado
        import copy
        state_copy = copy.copy(self._state)
        self._state_history.append(state_copy)
        
        # Limitar historial
        if len(self._state_history) > self._max_history:
            self._state_history = self._state_history[-self._max_history:]
    
    def set_mode(self, mode: SystemMode) -> bool:
        """
        Establece modo manualmente.
        
        Returns:
            True si se pudo establecer
        """
        if mode == SystemMode.EMERGENCY:
            # No permitir forzar emergencia si el sistema esta bien
            if self._state.is_healthy() and self._state.energy_level > 0.3:
                logger.warning("Cannot force emergency mode when system is healthy")
                return False
        
        old_mode = self._state.mode
        self._change_mode(old_mode, mode)
        return True
    
    def get_state(self) -> InternalState:
        """Obtiene estado actual."""
        return self._state
    
    def get_mode(self) -> SystemMode:
        """Obtiene modo actual."""
        return self._state.mode
    
    def start_interaction(self) -> None:
        """Marca inicio de interaccion."""
        self._state.interaction_active = True
        self._state.last_interaction_ns = time.time_ns()
        self._state.alertness = min(1.0, self._state.alertness + 0.2)
    
    def end_interaction(self) -> None:
        """Marca fin de interaccion."""
        self._state.interaction_active = False
    
    def boost_focus(self, amount: float = 0.2) -> None:
        """Aumenta el nivel de foco."""
        self._state.focus_level = min(1.0, self._state.focus_level + amount)
    
    def reduce_stress(self, amount: float = 0.2) -> None:
        """Reduce el nivel de estres."""
        self._state.stress_level = max(0.0, self._state.stress_level - amount)
    
    def on_mode_change(self, callback: Callable[[SystemMode, SystemMode], None]) -> None:
        """Registra callback para cambios de modo."""
        self._on_mode_change.append(callback)
    
    def on_alert(self, callback: Callable[[str, float], None]) -> None:
        """Registra callback para alertas."""
        self._on_alert.append(callback)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del regulador."""
        return {
            "current_state": self._state.to_dict(),
            "history_size": len(self._state_history),
            "thresholds": self.thresholds,
        }
