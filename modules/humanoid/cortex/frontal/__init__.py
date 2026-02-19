"""
Lóbulo Frontal Atlas: Planificación, decisión, control ejecutivo y motor concurrente.

Análogo biológico: Corteza prefrontal + área premotora
- TaskPlanner: Planificación de tareas y secuenciación
- DecisionMaker: Evaluación de opciones y toma de decisiones
- InhibitoryControl: Bloqueo de acciones peligrosas o inapropiadas
- ConcurrentGoalEngine: Motor de metas concurrente (CGE)
- ResourceArbiter: Arbitraje de recursos entre goals
"""
from .task_planner import TaskPlanner, CortexTaskPlanner
from .decision_maker import DecisionMaker
from .inhibitory_control import InhibitoryControl
from .concurrent_engine import ConcurrentGoalEngine, get_engine, cge_tick
from .resource_arbiter import ResourceArbiter
from .goal_context import GoalContext, GoalContextDB, CGEGoalStatus, CGEGoalPriority
from .parallel_executor import ParallelExecutor, StepResult

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
