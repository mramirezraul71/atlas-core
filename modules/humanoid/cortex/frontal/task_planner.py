"""
TaskPlanner: Planificador de tareas del lóbulo frontal.

Análogo biológico: Corteza prefrontal dorsolateral
- Descomposición de objetivos en pasos
- Planificación simbólica + LLM
- Secuenciación temporal
"""
from __future__ import annotations

import asyncio
import logging
import re
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class TaskStep:
    """Paso individual de un plan."""
    id: str
    description: str
    action_type: str  # "navigate", "grasp", "place", "speak", "look", "wait"
    parameters: Dict[str, Any] = field(default_factory=dict)
    dependencies: List[str] = field(default_factory=list)  # IDs de pasos previos
    estimated_duration_s: float = 5.0
    status: str = "pending"  # pending, active, completed, failed, skipped
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None


@dataclass
class TaskPlan:
    """Plan completo de tarea."""
    id: str
    goal: str
    steps: List[TaskStep]
    created_at: float = field(default_factory=lambda: __import__('time').time())
    status: str = "pending"  # pending, active, completed, failed, cancelled
    current_step_index: int = 0
    
    def is_valid(self) -> bool:
        """Verifica si el plan es válido."""
        return len(self.steps) > 0
    
    def current_step(self) -> Optional[TaskStep]:
        """Obtiene paso actual."""
        if 0 <= self.current_step_index < len(self.steps):
            return self.steps[self.current_step_index]
        return None
    
    def next_step(self) -> Optional[TaskStep]:
        """Avanza al siguiente paso."""
        self.current_step_index += 1
        return self.current_step()
    
    def mark_step_complete(self, result: Dict[str, Any] = None) -> None:
        """Marca paso actual como completado."""
        step = self.current_step()
        if step:
            step.status = "completed"
            step.result = result
    
    def mark_step_failed(self, error: str) -> None:
        """Marca paso actual como fallido."""
        step = self.current_step()
        if step:
            step.status = "failed"
            step.error = error
    
    def progress(self) -> float:
        """Calcula progreso del plan (0-1)."""
        if not self.steps:
            return 0.0
        completed = sum(1 for s in self.steps if s.status == "completed")
        return completed / len(self.steps)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario."""
        return {
            "id": self.id,
            "goal": self.goal,
            "steps": [
                {
                    "id": s.id,
                    "description": s.description,
                    "action_type": s.action_type,
                    "parameters": s.parameters,
                    "status": s.status,
                }
                for s in self.steps
            ],
            "status": self.status,
            "progress": self.progress(),
        }


class PDDLPlanner:
    """
    Planificador simbólico basado en PDDL.
    
    Para tareas estructuradas con precondiciones y efectos claros.
    """
    
    # Definición simplificada de acciones
    ACTIONS = {
        "navigate": {
            "preconditions": ["robot_at(?from)"],
            "effects": ["robot_at(?to)", "not robot_at(?from)"],
            "duration": 10.0,
        },
        "grasp": {
            "preconditions": ["robot_at(?location)", "object_at(?obj, ?location)", "hand_empty"],
            "effects": ["holding(?obj)", "not object_at(?obj, ?location)", "not hand_empty"],
            "duration": 5.0,
        },
        "place": {
            "preconditions": ["robot_at(?location)", "holding(?obj)"],
            "effects": ["object_at(?obj, ?location)", "hand_empty", "not holding(?obj)"],
            "duration": 3.0,
        },
        "look": {
            "preconditions": [],
            "effects": ["looked_at(?target)"],
            "duration": 1.0,
        },
        "speak": {
            "preconditions": [],
            "effects": ["said(?message)"],
            "duration": 2.0,
        },
    }
    
    def __init__(self):
        self.domain_actions = self.ACTIONS.copy()
    
    def plan(self, goal: str, world_state: Any) -> Optional[TaskPlan]:
        """
        Genera plan simbólico para objetivo.
        
        Args:
            goal: Objetivo en lenguaje natural o PDDL
            world_state: Estado actual del mundo
        
        Returns:
            TaskPlan si se puede planificar, None si no
        """
        # Parsear objetivo
        goal_type, params = self._parse_goal(goal)
        
        if goal_type == "fetch":
            return self._plan_fetch(params, world_state)
        elif goal_type == "place":
            return self._plan_place(params, world_state)
        elif goal_type == "navigate":
            return self._plan_navigate(params, world_state)
        
        # No se puede planificar simbólicamente
        return None
    
    def _parse_goal(self, goal: str) -> tuple:
        """Parsea objetivo a tipo + parámetros."""
        goal_lower = goal.lower()
        
        # Patrones de fetch
        fetch_patterns = [
            r"(?:trae|traeme|dame|busca|fetch|get|bring)\s+(?:la |el |un |una )?(.+?)(?:\s+de\s+|\s+from\s+)?(.+)?",
        ]
        for pattern in fetch_patterns:
            match = re.search(pattern, goal_lower)
            if match:
                return "fetch", {"object": match.group(1).strip(), "location": match.group(2).strip() if match.group(2) else None}
        
        # Patrones de place
        place_patterns = [
            r"(?:pon|coloca|deja|place|put)\s+(?:la |el )?(.+?)\s+(?:en|on|at)\s+(.+)",
        ]
        for pattern in place_patterns:
            match = re.search(pattern, goal_lower)
            if match:
                return "place", {"object": match.group(1).strip(), "location": match.group(2).strip()}
        
        # Patrones de navigate
        navigate_patterns = [
            r"(?:ve|ir|navega|go|move)\s+(?:a|to|hacia)\s+(.+)",
        ]
        for pattern in navigate_patterns:
            match = re.search(pattern, goal_lower)
            if match:
                return "navigate", {"location": match.group(1).strip()}
        
        return "unknown", {"raw": goal}
    
    def _plan_fetch(self, params: Dict, world_state: Any) -> TaskPlan:
        """Genera plan para buscar objeto."""
        obj = params.get("object", "objeto")
        location = params.get("location", "unknown")
        
        steps = [
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Localizar {obj}",
                action_type="look",
                parameters={"target": obj},
                estimated_duration_s=2.0,
            ),
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Navegar hacia {obj}",
                action_type="navigate",
                parameters={"target": location or obj},
                estimated_duration_s=10.0,
            ),
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Agarrar {obj}",
                action_type="grasp",
                parameters={"object": obj},
                estimated_duration_s=5.0,
            ),
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description="Regresar al punto inicial",
                action_type="navigate",
                parameters={"target": "user"},
                estimated_duration_s=10.0,
            ),
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Entregar {obj}",
                action_type="place",
                parameters={"object": obj, "target": "user_hand"},
                estimated_duration_s=3.0,
            ),
        ]
        
        # Agregar dependencias secuenciales
        for i in range(1, len(steps)):
            steps[i].dependencies = [steps[i-1].id]
        
        return TaskPlan(
            id=f"plan_{uuid.uuid4().hex[:8]}",
            goal=f"fetch {obj}",
            steps=steps,
        )
    
    def _plan_place(self, params: Dict, world_state: Any) -> TaskPlan:
        """Genera plan para colocar objeto."""
        obj = params.get("object", "objeto")
        location = params.get("location", "mesa")
        
        steps = [
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Navegar hacia {location}",
                action_type="navigate",
                parameters={"target": location},
                estimated_duration_s=10.0,
            ),
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Colocar {obj} en {location}",
                action_type="place",
                parameters={"object": obj, "location": location},
                estimated_duration_s=3.0,
            ),
        ]
        
        steps[1].dependencies = [steps[0].id]
        
        return TaskPlan(
            id=f"plan_{uuid.uuid4().hex[:8]}",
            goal=f"place {obj} on {location}",
            steps=steps,
        )
    
    def _plan_navigate(self, params: Dict, world_state: Any) -> TaskPlan:
        """Genera plan de navegación."""
        location = params.get("location", "destino")
        
        steps = [
            TaskStep(
                id=f"step_{uuid.uuid4().hex[:8]}",
                description=f"Navegar a {location}",
                action_type="navigate",
                parameters={"target": location},
                estimated_duration_s=15.0,
            ),
        ]
        
        return TaskPlan(
            id=f"plan_{uuid.uuid4().hex[:8]}",
            goal=f"navigate to {location}",
            steps=steps,
        )


class TaskPlanner:
    """
    Planificador de tareas del lóbulo frontal.
    
    Combina planificación simbólica (PDDL) con planificación basada en LLM.
    """
    
    def __init__(self, llm_client: Any = None):
        """
        Inicializa el planificador.
        
        Args:
            llm_client: Cliente LLM opcional para planificación neuronal
        """
        self.symbolic_planner = PDDLPlanner()
        self.llm_client = llm_client
        self._plan_cache: Dict[str, TaskPlan] = {}
    
    async def plan(self, goal: str, world_state: Any = None, context: Dict = None) -> TaskPlan:
        """
        Genera plan para objetivo.
        
        Intenta planificación simbólica primero (rápida, determinista).
        Si falla, usa LLM para planificación neuronal.
        
        Args:
            goal: Objetivo en lenguaje natural
            world_state: Estado actual del mundo
            context: Contexto adicional
        
        Returns:
            TaskPlan con pasos a ejecutar
        """
        context = context or {}
        
        # 1. Intentar planificación simbólica
        try:
            symbolic_plan = self.symbolic_planner.plan(goal, world_state)
            if symbolic_plan and symbolic_plan.is_valid():
                logger.info(f"Symbolic plan generated for: {goal}")
                return symbolic_plan
        except Exception as e:
            logger.debug(f"Symbolic planning failed: {e}")
        
        # 2. Planificación con LLM
        if self.llm_client:
            try:
                llm_plan = await self._plan_with_llm(goal, world_state, context)
                if llm_plan and llm_plan.is_valid():
                    logger.info(f"LLM plan generated for: {goal}")
                    return llm_plan
            except Exception as e:
                logger.error(f"LLM planning failed: {e}")
        
        # 3. Fallback: plan simple
        return self._fallback_plan(goal)
    
    async def _plan_with_llm(self, goal: str, world_state: Any, context: Dict) -> TaskPlan:
        """Genera plan usando LLM."""
        prompt = self._build_planning_prompt(goal, world_state, context)
        
        response = await self.llm_client.generate(prompt)
        return self._parse_llm_response(goal, response)
    
    def _build_planning_prompt(self, goal: str, world_state: Any, context: Dict) -> str:
        """Construye prompt para planificación con LLM."""
        world_desc = ""
        if world_state:
            if hasattr(world_state, 'to_dict'):
                world_desc = f"\nEstado del mundo: {world_state.to_dict()}"
            elif isinstance(world_state, dict):
                world_desc = f"\nEstado del mundo: {world_state}"
        
        context_desc = ""
        if context:
            context_desc = f"\nContexto: {context}"
        
        return f"""Eres un planificador de tareas para un robot humanoide.

Objetivo: {goal}
{world_desc}
{context_desc}

Genera un plan paso a paso. Cada paso debe tener:
- Número del paso
- Descripción breve
- Tipo de acción: navigate, grasp, place, look, speak, wait

Formato:
1. [navigate] Descripción del paso
2. [grasp] Descripción del paso
...

Responde SOLO con los pasos numerados."""
    
    def _parse_llm_response(self, goal: str, response: str) -> TaskPlan:
        """Parsea respuesta del LLM a TaskPlan."""
        steps = []
        
        # Parsear líneas numeradas
        lines = response.strip().split('\n')
        for line in lines:
            # Buscar patrón: N. [tipo] descripción
            match = re.match(r'^\d+\.\s*\[(\w+)\]\s*(.+)$', line.strip())
            if match:
                action_type = match.group(1).lower()
                description = match.group(2).strip()
                
                steps.append(TaskStep(
                    id=f"step_{uuid.uuid4().hex[:8]}",
                    description=description,
                    action_type=action_type,
                    estimated_duration_s=5.0,
                ))
        
        # Agregar dependencias secuenciales
        for i in range(1, len(steps)):
            steps[i].dependencies = [steps[i-1].id]
        
        return TaskPlan(
            id=f"plan_{uuid.uuid4().hex[:8]}",
            goal=goal,
            steps=steps,
        )
    
    def _fallback_plan(self, goal: str) -> TaskPlan:
        """Plan de fallback cuando no se puede planificar."""
        return TaskPlan(
            id=f"plan_{uuid.uuid4().hex[:8]}",
            goal=goal,
            steps=[
                TaskStep(
                    id=f"step_{uuid.uuid4().hex[:8]}",
                    description=f"Ejecutar: {goal}",
                    action_type="execute",
                    parameters={"goal": goal},
                    estimated_duration_s=30.0,
                )
            ],
        )


# Alias para compatibilidad
CortexTaskPlanner = TaskPlanner
