"""
Lóbulo Frontal Atlas: Planificación, decisión, control ejecutivo y motor concurrente.

Análogo biológico: Corteza prefrontal + área premotora
- TaskPlanner: Planificación de tareas y secuenciación
- DecisionMaker: Evaluación de opciones y toma de decisiones
- InhibitoryControl: Bloqueo de acciones peligrosas o inapropiadas
- ConcurrentGoalEngine: Motor de metas concurrente (CGE)
- ResourceArbiter: Arbitraje de recursos entre goals
"""
from .concurrent_engine import ConcurrentGoalEngine, cge_tick, get_engine
from .decision_maker import DecisionMaker
from .goal_context import (CGEGoalPriority, CGEGoalStatus, GoalContext,
                           GoalContextDB)
from .inhibitory_control import InhibitoryControl
from .parallel_executor import ParallelExecutor, StepResult
from .resource_arbiter import ResourceArbiter
from .task_planner import CortexTaskPlanner, TaskPlanner

__all__ = [
    "TaskPlanner",
    "CortexTaskPlanner",
    "DecisionMaker",
    "InhibitoryControl",
    "ConcurrentGoalEngine",
    "get_engine",
    "cge_tick",
    "ResourceArbiter",
    "GoalContext",
    "GoalContextDB",
    "CGEGoalStatus",
    "CGEGoalPriority",
    "ParallelExecutor",
    "StepResult",
]
