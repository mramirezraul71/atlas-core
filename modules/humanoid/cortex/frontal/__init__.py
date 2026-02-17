"""
Lóbulo Frontal Atlas: Planificación, decisión y control ejecutivo.

Análogo biológico: Corteza prefrontal + área premotora
- TaskPlanner: Planificación de tareas y secuenciación
- DecisionMaker: Evaluación de opciones y toma de decisiones
- InhibitoryControl: Bloqueo de acciones peligrosas o inapropiadas
"""
from .task_planner import TaskPlanner, CortexTaskPlanner
from .decision_maker import DecisionMaker
from .inhibitory_control import InhibitoryControl

__all__ = [
    "TaskPlanner",
    "CortexTaskPlanner",
    "DecisionMaker",
    "InhibitoryControl",
]
