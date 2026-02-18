"""
RewardEngine: Motor de recompensas del sistema limbico.

Analogo biologico: Nucleo accumbens + sistema dopaminergico
- Calculo de senales de recompensa
- Refuerzo de comportamientos
- Funcion de utilidad
"""
from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="limbic")
    except Exception:
        pass


@dataclass
class RewardSignal:
    """Senal de recompensa."""
    value: float  # -1.0 a 1.0
    source: str  # "goal", "feedback", "internal", "learning"
    reason: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    context: Dict[str, Any] = field(default_factory=dict)
    
    def is_positive(self) -> bool:
        return self.value > 0
    
    def is_negative(self) -> bool:
        return self.value < 0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "value": self.value,
            "source": self.source,
            "reason": self.reason,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass
class RewardComponent:
    """Componente de la funcion de recompensa."""
    name: str
    weight: float
    calculator: Callable[[Dict[str, Any]], float]
    enabled: bool = True


class RewardEngine:
    """
    Motor de recompensas del sistema limbico.
    
    Calcula senales de recompensa basadas en:
    - Cumplimiento de objetivos
    - Feedback del usuario
    - Metricas internas (eficiencia, seguridad)
    - Aprendizaje por refuerzo
    """
    
    def __init__(self, history_size: int = 1000):
        """
        Inicializa el motor de recompensas.
        
        Args:
            history_size: Tamano del historial de recompensas
        """
        self.history_size = history_size
        
        # Historial de recompensas
        self._history: deque = deque(maxlen=history_size)
        
        # Componentes de la funcion de recompensa
        self._components: Dict[str, RewardComponent] = {}
        
        # Callbacks para notificaciones
        self._callbacks: List[Callable[[RewardSignal], None]] = []
        
        # Estadisticas
        self._total_rewards = 0.0
        self._reward_count = 0
        
        # Registrar componentes por defecto
        self._register_default_components()
    
    def _register_default_components(self) -> None:
        """Registra componentes de recompensa por defecto."""
        
        # Componente: Cumplimiento de objetivo
        self.add_component(RewardComponent(
            name="goal_completion",
            weight=1.0,
            calculator=self._calc_goal_completion,
        ))
        
        # Componente: Eficiencia (tiempo)
        self.add_component(RewardComponent(
            name="efficiency",
            weight=0.3,
            calculator=self._calc_efficiency,
        ))
        
        # Componente: Seguridad
        self.add_component(RewardComponent(
            name="safety",
            weight=0.5,
            calculator=self._calc_safety,
        ))
        
        # Componente: Interaccion positiva
        self.add_component(RewardComponent(
            name="interaction",
            weight=0.4,
            calculator=self._calc_interaction,
        ))
    
    def add_component(self, component: RewardComponent) -> None:
        """Agrega componente a la funcion de recompensa."""
        self._components[component.name] = component
    
    def remove_component(self, name: str) -> bool:
        """Remueve componente por nombre."""
        if name in self._components:
            del self._components[name]
            return True
        return False
    
    def set_component_weight(self, name: str, weight: float) -> bool:
        """Ajusta peso de un componente."""
        if name in self._components:
            self._components[name].weight = weight
            return True
        return False
    
    def calculate_reward(self, context: Dict[str, Any]) -> RewardSignal:
        """
        Calcula recompensa total basada en contexto.
        
        Args:
            context: Diccionario con informacion del contexto:
                - goal_completed: bool
                - goal_progress: float
                - elapsed_time_ms: float
                - expected_time_ms: float
                - safety_violations: int
                - user_feedback: float (-1 a 1)
                - interaction_quality: float
        
        Returns:
            RewardSignal con valor calculado
        """
        total_reward = 0.0
        total_weight = 0.0
        reasons = []
        
        for name, component in self._components.items():
            if not component.enabled:
                continue
            
            try:
                value = component.calculator(context)
                weighted = value * component.weight
                total_reward += weighted
                total_weight += component.weight
                
                if abs(value) > 0.1:
                    reasons.append(f"{name}:{value:.2f}")
            except Exception as e:
                logger.error(f"Error in reward component {name}: {e}")
        
        # Normalizar
        if total_weight > 0:
            total_reward /= total_weight
        
        # Limitar a [-1, 1]
        total_reward = max(-1.0, min(1.0, total_reward))
        
        signal = RewardSignal(
            value=total_reward,
            source="computed",
            reason=", ".join(reasons) if reasons else "neutral",
            context=context,
        )
        
        # Log significant rewards (high positive or negative)
        task_id = context.get("task_id", context.get("goal_id", "unknown"))
        if abs(total_reward) > 0.3:
            _bitacora(f"Reward: {total_reward:.2f} for {task_id}", ok=total_reward > 0)
        
        self._record_reward(signal)
        return signal
    
    def emit_reward(self, value: float, source: str, reason: str,
                   context: Dict[str, Any] = None) -> RewardSignal:
        """
        Emite una senal de recompensa directa.
        
        Args:
            value: Valor de recompensa (-1 a 1)
            source: Origen de la recompensa
            reason: Razon de la recompensa
            context: Contexto adicional
        
        Returns:
            RewardSignal emitida
        """
        signal = RewardSignal(
            value=max(-1.0, min(1.0, value)),
            source=source,
            reason=reason,
            context=context or {},
        )
        
        # Log significant rewards (high positive or negative)
        task_id = context.get("task_id", context.get("goal_id", "unknown")) if context else "unknown"
        if abs(value) > 0.3:
            _bitacora(f"Reward: {value:.2f} for {task_id}", ok=value > 0)
        
        self._record_reward(signal)
        return signal
    
    def emit_goal_reward(self, success: bool, 
                        progress: float = 1.0,
                        efficiency: float = 1.0) -> RewardSignal:
        """
        Emite recompensa por resultado de objetivo.
        
        Args:
            success: Si el objetivo fue exitoso
            progress: Progreso alcanzado (0-1)
            efficiency: Eficiencia (tiempo real / esperado)
        
        Returns:
            RewardSignal
        """
        if success:
            base_reward = 0.5 + (0.5 * progress)
            # Bonus por eficiencia
            if efficiency > 1.0:
                base_reward += 0.1 * min(efficiency - 1.0, 0.5)
            reason = "goal_success"
        else:
            base_reward = -0.3 - (0.3 * (1.0 - progress))
            reason = "goal_failure"
        
        signal = self.emit_reward(base_reward, "goal", reason, {
            "success": success,
            "progress": progress,
            "efficiency": efficiency,
        })
        
        # Log significant goal rewards
        if abs(base_reward) > 0.3:
            task_id = f"goal_{reason}"
            _bitacora(f"Reward: {base_reward:.2f} for {task_id}", ok=success)
        
        return signal
    
    def emit_user_feedback(self, positive: bool, 
                          intensity: float = 1.0) -> RewardSignal:
        """
        Emite recompensa por feedback del usuario.
        
        Args:
            positive: Si el feedback es positivo
            intensity: Intensidad del feedback (0-1)
        
        Returns:
            RewardSignal
        """
        value = intensity if positive else -intensity
        value *= 0.8  # Factor de escalado
        
        return self.emit_reward(value, "feedback", 
                               "positive_feedback" if positive else "negative_feedback",
                               {"intensity": intensity})
    
    def emit_safety_penalty(self, severity: float, 
                           violation_type: str) -> RewardSignal:
        """
        Emite penalizacion por violacion de seguridad.
        
        Args:
            severity: Severidad (0-1)
            violation_type: Tipo de violacion
        
        Returns:
            RewardSignal
        """
        value = -severity * 0.9  # Penalizacion fuerte
        
        return self.emit_reward(value, "safety", 
                               f"safety_violation:{violation_type}",
                               {"severity": severity, "type": violation_type})
    
    def _record_reward(self, signal: RewardSignal) -> None:
        """Registra recompensa en historial."""
        self._history.append(signal)
        self._total_rewards += signal.value
        self._reward_count += 1
        
        # Notificar callbacks
        for callback in self._callbacks:
            try:
                callback(signal)
            except Exception as e:
                logger.error(f"Error in reward callback: {e}")
    
    def on_reward(self, callback: Callable[[RewardSignal], None]) -> None:
        """Registra callback para recibir recompensas."""
        self._callbacks.append(callback)
    
    # === Calculadores de Componentes ===
    
    def _calc_goal_completion(self, context: Dict[str, Any]) -> float:
        """Calcula recompensa por cumplimiento de objetivo."""
        if context.get("goal_completed"):
            return 1.0
        
        progress = context.get("goal_progress", 0.0)
        return progress * 0.5  # Recompensa parcial por progreso
    
    def _calc_efficiency(self, context: Dict[str, Any]) -> float:
        """Calcula recompensa por eficiencia."""
        elapsed = context.get("elapsed_time_ms", 0)
        expected = context.get("expected_time_ms", 0)
        
        if not expected or not elapsed:
            return 0.0
        
        ratio = expected / elapsed if elapsed > 0 else 1.0
        
        if ratio >= 1.0:
            # Mas rapido de lo esperado: bonus
            return min((ratio - 1.0) * 0.5, 0.5)
        else:
            # Mas lento: penalizacion leve
            return max((ratio - 1.0) * 0.3, -0.3)
    
    def _calc_safety(self, context: Dict[str, Any]) -> float:
        """Calcula recompensa/penalizacion por seguridad."""
        violations = context.get("safety_violations", 0)
        
        if violations == 0:
            return 0.2  # Bonus por no tener violaciones
        
        # Penalizacion progresiva
        return -min(violations * 0.3, 1.0)
    
    def _calc_interaction(self, context: Dict[str, Any]) -> float:
        """Calcula recompensa por calidad de interaccion."""
        user_feedback = context.get("user_feedback", 0.0)
        interaction_quality = context.get("interaction_quality", 0.5)
        
        return (user_feedback * 0.6 + interaction_quality * 0.4)
    
    # === Estadisticas ===
    
    def get_recent_rewards(self, n: int = 10) -> List[RewardSignal]:
        """Obtiene las ultimas n recompensas."""
        return list(self._history)[-n:]
    
    def get_average_reward(self, window: int = None) -> float:
        """Obtiene recompensa promedio."""
        if not self._history:
            return 0.0
        
        if window:
            rewards = list(self._history)[-window:]
        else:
            rewards = list(self._history)
        
        return sum(r.value for r in rewards) / len(rewards)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del motor."""
        return {
            "total_rewards": self._total_rewards,
            "reward_count": self._reward_count,
            "average_reward": self._total_rewards / self._reward_count if self._reward_count > 0 else 0,
            "recent_average": self.get_average_reward(100),
            "history_size": len(self._history),
            "components": list(self._components.keys()),
        }
    
    def reset_stats(self) -> None:
        """Resetea estadisticas."""
        self._total_rewards = 0.0
        self._reward_count = 0
        self._history.clear()
