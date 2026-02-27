"""
GoalManager: Gestor de objetivos del sistema limbico.

Analogo biologico: Amigdala + corteza cingulada anterior
- Gestion de objetivos activos
- Priorizacion dinamica
- Seleccion de objetivo actual
"""
from __future__ import annotations

import logging
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="limbic")
    except Exception:
        pass


class GoalStatus(Enum):
    """Estado de un objetivo."""
    PENDING = "pending"
    ACTIVE = "active"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class GoalPriority(Enum):
    """Nivel de prioridad de objetivo."""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    URGENT = 3
    CRITICAL = 4  # Solo para emergencias


@dataclass
class Goal:
    """Representa un objetivo del robot."""
    id: str
    goal_type: str  # "fetch", "navigate", "speak", "observe", "learn", etc.
    description: str
    target: str  # Objeto, ubicacion, persona objetivo
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    # Prioridad y urgencia
    priority: GoalPriority = GoalPriority.NORMAL
    deadline_ns: Optional[int] = None
    
    # Estado
    status: GoalStatus = GoalStatus.PENDING
    progress: float = 0.0  # 0.0 - 1.0
    
    # Timestamps
    created_at_ns: int = field(default_factory=lambda: time.time_ns())
    started_at_ns: Optional[int] = None
    completed_at_ns: Optional[int] = None
    
    # Origen
    source: str = "user"  # "user", "system", "learning", "reactive"
    parent_goal_id: Optional[str] = None  # Para subgoals
    
    # Resultados
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    reward: float = 0.0
    
    def is_active(self) -> bool:
        return self.status == GoalStatus.ACTIVE
    
    def is_complete(self) -> bool:
        return self.status in (GoalStatus.COMPLETED, GoalStatus.FAILED, GoalStatus.CANCELLED)
    
    def is_urgent(self) -> bool:
        if self.priority.value >= GoalPriority.URGENT.value:
            return True
        if self.deadline_ns and time.time_ns() > self.deadline_ns:
            return True
        return False
    
    def elapsed_ms(self) -> float:
        """Tiempo transcurrido desde inicio."""
        if not self.started_at_ns:
            return 0.0
        end = self.completed_at_ns or time.time_ns()
        return (end - self.started_at_ns) / 1e6
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "goal_type": self.goal_type,
            "description": self.description,
            "target": self.target,
            "parameters": self.parameters,
            "priority": self.priority.name,
            "status": self.status.name,
            "progress": self.progress,
            "source": self.source,
            "elapsed_ms": self.elapsed_ms(),
            "reward": self.reward,
        }


class GoalManager:
    """
    Gestor de objetivos del sistema limbico.
    
    Responsabilidades:
    - Mantener lista de objetivos activos
    - Priorizar objetivos segun urgencia e importancia
    - Seleccionar objetivo actual a perseguir
    - Rastrear progreso y completitud
    """
    
    def __init__(self, max_active_goals: int = 10):
        """
        Inicializa el gestor de objetivos.
        
        Args:
            max_active_goals: Maximo de objetivos activos simultaneos
        """
        self.max_active_goals = max_active_goals
        
        # Almacenamiento de objetivos
        self._goals: Dict[str, Goal] = {}
        
        # Objetivo actualmente en foco
        self._current_goal_id: Optional[str] = None
        
        # Callbacks
        self._on_goal_completed: List[Callable[[Goal], None]] = []
        self._on_goal_failed: List[Callable[[Goal], None]] = []
        self._on_goal_started: List[Callable[[Goal], None]] = []
    
    def add_goal(self, 
                goal_type: str,
                description: str,
                target: str,
                priority: GoalPriority = GoalPriority.NORMAL,
                parameters: Dict[str, Any] = None,
                source: str = "user",
                parent_goal_id: str = None,
                deadline_ns: int = None) -> Goal:
        """
        Agrega un nuevo objetivo.
        
        Returns:
            Goal creado
        """
        goal_id = f"goal_{uuid.uuid4().hex[:12]}"
        
        goal = Goal(
            id=goal_id,
            goal_type=goal_type,
            description=description,
            target=target,
            parameters=parameters or {},
            priority=priority,
            source=source,
            parent_goal_id=parent_goal_id,
            deadline_ns=deadline_ns,
        )
        
        self._goals[goal_id] = goal
        logger.info(f"Goal added: {goal_id} - {description}")
        _bitacora(f"Goal añadido: {goal.description} prioridad={goal.priority.name}")
        
        # Auto-activar si es de alta prioridad y no hay objetivo actual
        if priority.value >= GoalPriority.HIGH.value:
            if not self._current_goal_id or self._should_preempt(goal):
                self.activate_goal(goal_id)
        
        return goal
    
    def get_goal(self, goal_id: str) -> Optional[Goal]:
        """Obtiene un objetivo por ID."""
        return self._goals.get(goal_id)
    
    def get_current_goal(self) -> Optional[Goal]:
        """Obtiene el objetivo actualmente en foco."""
        if self._current_goal_id:
            return self._goals.get(self._current_goal_id)
        return None
    
    def get_active_goals(self) -> List[Goal]:
        """Obtiene todos los objetivos activos."""
        return [g for g in self._goals.values() if g.status == GoalStatus.ACTIVE]
    
    def get_pending_goals(self) -> List[Goal]:
        """Obtiene objetivos pendientes ordenados por prioridad."""
        pending = [g for g in self._goals.values() if g.status == GoalStatus.PENDING]
        return sorted(pending, key=lambda g: (-g.priority.value, g.created_at_ns))
    
    def activate_goal(self, goal_id: str) -> bool:
        """
        Activa un objetivo (lo pone en foco).
        
        Returns:
            True si se activo exitosamente
        """
        goal = self._goals.get(goal_id)
        if not goal:
            return False
        
        if goal.is_complete():
            return False
        
        # Pausar objetivo actual si existe
        if self._current_goal_id and self._current_goal_id != goal_id:
            current = self._goals.get(self._current_goal_id)
            if current and current.is_active():
                current.status = GoalStatus.PAUSED
        
        # Activar nuevo objetivo
        goal.status = GoalStatus.ACTIVE
        if not goal.started_at_ns:
            goal.started_at_ns = time.time_ns()
        
        self._current_goal_id = goal_id
        logger.info(f"Goal activated: {goal_id}")
        
        # Notificar
        for callback in self._on_goal_started:
            try:
                callback(goal)
            except Exception as e:
                logger.error(f"Error in goal started callback: {e}")
        
        return True
    
    def update_progress(self, goal_id: str, progress: float) -> bool:
        """
        Actualiza el progreso de un objetivo.
        
        Args:
            goal_id: ID del objetivo
            progress: Progreso (0.0 - 1.0)
        
        Returns:
            True si se actualizo
        """
        goal = self._goals.get(goal_id)
        if not goal:
            return False
        
        goal.progress = max(0.0, min(1.0, progress))
        
        # Auto-completar si llega a 100%
        if goal.progress >= 1.0 and goal.status == GoalStatus.ACTIVE:
            self.complete_goal(goal_id)
        
        return True
    
    def complete_goal(self, goal_id: str, 
                     result: Dict[str, Any] = None,
                     reward: float = 1.0) -> bool:
        """
        Marca un objetivo como completado.
        
        Args:
            goal_id: ID del objetivo
            result: Resultado opcional
            reward: Recompensa (0-1)
        
        Returns:
            True si se completo
        """
        goal = self._goals.get(goal_id)
        if not goal:
            return False
        
        goal.status = GoalStatus.COMPLETED
        goal.completed_at_ns = time.time_ns()
        goal.progress = 1.0
        goal.result = result
        goal.reward = reward
        
        logger.info(f"Goal completed: {goal_id}, reward={reward}")
        _bitacora(f"Goal completado: {goal.description}")
        
        # Si era el objetivo actual, seleccionar siguiente
        if self._current_goal_id == goal_id:
            self._current_goal_id = None
            self._select_next_goal()
        
        # Notificar
        for callback in self._on_goal_completed:
            try:
                callback(goal)
            except Exception as e:
                logger.error(f"Error in goal completed callback: {e}")
        
        return True
    
    def fail_goal(self, goal_id: str, error: str = None) -> bool:
        """
        Marca un objetivo como fallido.
        
        Args:
            goal_id: ID del objetivo
            error: Mensaje de error
        
        Returns:
            True si se marco como fallido
        """
        goal = self._goals.get(goal_id)
        if not goal:
            return False
        
        goal.status = GoalStatus.FAILED
        goal.completed_at_ns = time.time_ns()
        goal.error = error
        goal.reward = -0.5  # Penalizacion
        
        logger.warning(f"Goal failed: {goal_id}, error={error}")
        _bitacora(f"Goal falló: {goal.description}", ok=False)
        
        # Si era el objetivo actual, seleccionar siguiente
        if self._current_goal_id == goal_id:
            self._current_goal_id = None
            self._select_next_goal()
        
        # Notificar
        for callback in self._on_goal_failed:
            try:
                callback(goal)
            except Exception as e:
                logger.error(f"Error in goal failed callback: {e}")
        
        return True
    
    def cancel_goal(self, goal_id: str, reason: str = None) -> bool:
        """Cancela un objetivo."""
        goal = self._goals.get(goal_id)
        if not goal:
            return False
        
        goal.status = GoalStatus.CANCELLED
        goal.completed_at_ns = time.time_ns()
        goal.error = reason or "Cancelled"
        
        if self._current_goal_id == goal_id:
            self._current_goal_id = None
            self._select_next_goal()
        
        return True
    
    def _should_preempt(self, new_goal: Goal) -> bool:
        """Determina si un nuevo objetivo debe reemplazar al actual."""
        current = self.get_current_goal()
        if not current:
            return True
        
        # Siempre ceder ante objetivos criticos
        if new_goal.priority == GoalPriority.CRITICAL:
            return True
        
        # Ceder si el nuevo es mas urgente
        if new_goal.priority.value > current.priority.value + 1:
            return True
        
        # Ceder si el actual esta casi completo
        if current.progress > 0.9:
            return False  # Dejar terminar
        
        return False
    
    def _select_next_goal(self) -> None:
        """Selecciona el siguiente objetivo a activar."""
        # Primero buscar objetivos pausados de alta prioridad
        paused = [g for g in self._goals.values() if g.status == GoalStatus.PAUSED]
        if paused:
            best = max(paused, key=lambda g: g.priority.value)
            self.activate_goal(best.id)
            return
        
        # Luego objetivos pendientes
        pending = self.get_pending_goals()
        if pending:
            self.activate_goal(pending[0].id)
    
    def on_goal_completed(self, callback: Callable[[Goal], None]) -> None:
        """Registra callback para objetivos completados."""
        self._on_goal_completed.append(callback)
    
    def on_goal_failed(self, callback: Callable[[Goal], None]) -> None:
        """Registra callback para objetivos fallidos."""
        self._on_goal_failed.append(callback)
    
    def on_goal_started(self, callback: Callable[[Goal], None]) -> None:
        """Registra callback para objetivos iniciados."""
        self._on_goal_started.append(callback)
    
    def cleanup_old_goals(self, max_age_hours: int = 24) -> int:
        """
        Limpia objetivos completados antiguos.
        
        Returns:
            Numero de objetivos eliminados
        """
        cutoff = time.time_ns() - (max_age_hours * 3600 * 1e9)
        to_remove = []
        
        for goal_id, goal in self._goals.items():
            if goal.is_complete() and goal.completed_at_ns and goal.completed_at_ns < cutoff:
                to_remove.append(goal_id)
        
        for goal_id in to_remove:
            del self._goals[goal_id]
        
        return len(to_remove)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del gestor."""
        by_status = {}
        for goal in self._goals.values():
            status = goal.status.name
            by_status[status] = by_status.get(status, 0) + 1
        
        return {
            "total_goals": len(self._goals),
            "by_status": by_status,
            "current_goal_id": self._current_goal_id,
            "active_count": len(self.get_active_goals()),
            "pending_count": len(self.get_pending_goals()),
        }
