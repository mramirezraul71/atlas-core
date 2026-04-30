"""
LangFlow Orchestrator - Integración con LangFlow para orquestación de tareas complejas
Conecta con Unified Memory Cortex para persistencia y contexto
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    # Intentar imports para diferentes versiones de LangFlow
    try:
        from langflow import Flow
        from langflow.helpers.flow import import_flow

        LANGFLOW_AVAILABLE = True
        print("✅ LangFlow imports (new version)")
    except ImportError:
        try:
            # Versión anterior
            from langflow import Flow
            from langflow.helpers import import_flow

            LANGFLOW_AVAILABLE = True
            print("✅ LangFlow imports (legacy version)")
        except ImportError:
            # Versión más reciente - API diferente
            import langflow

            LANGFLOW_AVAILABLE = True
            Flow = None  # Se manejará dinámicamente
            import_flow = None
            print("✅ LangFlow disponible (API moderna)")
except ImportError:
    print("⚠️ LangFlow no disponible, usando modo simulación")
    LANGFLOW_AVAILABLE = False
    Flow = None
    import_flow = None


class LangFlowOrchestrator:
    """
    Orquestador de tareas usando LangFlow para encadenamiento complejo.

    Características:
    - Cargar y ejecutar flows LangFlow
    - Integración con memoria de Atlas
    - Persistencia de resultados
    - Contexto automático desde Unified Memory Cortex
    """

    def __init__(self, flows_dir: Optional[str] = None):
        if flows_dir is None:
            flows_dir = str(Path(__file__).resolve().parents[3] / "flows")

        self.flows_dir = Path(flows_dir)
        self.flows_dir.mkdir(parents=True, exist_ok=True)

        self.loaded_flows: Dict[str, Any] = {}
        self.memory_enabled = True
        self.langflow_available = LANGFLOW_AVAILABLE

        if self.langflow_available:
            print(f"✅ LangFlow Orchestrator iniciado en {self.flows_dir}")
        else:
            print(f"⚠️ LangFlow Orchestrator en modo simulación en {self.flows_dir}")

        # Cargar flows disponibles
        self._load_available_flows()

    def _load_available_flows(self):
        """Cargar todos los flows disponibles del directorio."""
        if not self.langflow_available:
            print("  ⚠️ LangFlow no disponible, no se pueden cargar flows")
            return

        # Para API moderna de LangFlow, cargar flows manualmente
        for flow_file in self.flows_dir.glob("*.json"):
            try:
                flow_name = flow_file.stem
                # Cargar flow desde JSON manualmente (manejar UTF-8 BOM)
                with open(flow_file, "r", encoding="utf-8-sig") as f:
                    flow_data = json.load(f)

                self.loaded_flows[flow_name] = {
                    "file_path": str(flow_file),
                    "data": flow_data,
                    "description": flow_data.get("description", ""),
                    "name": flow_data.get("name", flow_name),
                }
                print(
                    f"  📁 Flow cargado: {flow_name} ({flow_data.get('description', 'No description')})"
                )
            except Exception as e:
                print(f"  ❌ Error cargando flow {flow_file}: {e}")

    def create_task_chain(
        self,
        goal: str,
        context: Optional[Dict[str, Any]] = None,
        memory_types: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """
        Crear cadena de tareas para un objetivo específico.

        Args:
            goal: Objetivo principal
            context: Contexto adicional
            memory_types: Tipos de memoria a consultar

        Returns:
            Dict con información de la cadena creada
        """
        task_id = str(uuid.uuid4())

        # Obtener contexto desde memoria
        memory_context = (
            self._get_memory_context(goal, memory_types) if self.memory_enabled else {}
        )

        # Combinar contextos
        full_context = {
            "goal": goal,
            "timestamp": datetime.now().isoformat(),
            "task_id": task_id,
        }

        if context:
            full_context.update(context)

        if memory_context:
            full_context["memory_context"] = memory_context

        # Crear o seleccionar flow apropiado
        flow_name = self._select_flow_for_goal(goal)

        chain_info = {
            "task_id": task_id,
            "goal": goal,
            "flow_name": flow_name,
            "context": full_context,
            "status": "created",
            "created_at": datetime.now().isoformat(),
        }

        # Guardar en memoria
        self._save_task_chain(chain_info)

        return chain_info

    def execute_task_chain(
        self, task_id: str, inputs: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Ejecutar cadena de tareas.

        Args:
            task_id: ID de la tarea
            inputs: Inputs adicionales para el flow

        Returns:
            Resultados de la ejecución
        """
        # Recuperar información de la tarea
        task_info = self._get_task_chain(task_id)
        if not task_info:
            return {"task_id": task_id, "status": "error", "error": "Task not found"}

        flow_name = task_info["flow_name"]

        if not Flow or flow_name not in self.loaded_flows:
            # Modo simulación sin LangFlow real
            return self._simulate_execution(task_info, inputs)

        try:
            # Ejecutar flow LangFlow real
            flow_path = self.loaded_flows[flow_name]
            flow = import_flow(flow_path)

            # Preparar inputs
            flow_inputs = {
                "goal": task_info["goal"],
                "context": task_info["context"],
            }

            if inputs:
                flow_inputs.update(inputs)

            # Ejecutar
            results = flow.run(flow_inputs)

            # Guardar resultados en memoria
            execution_result = {
                "task_id": task_id,
                "status": "completed",
                "results": results,
                "executed_at": datetime.now().isoformat(),
                "flow_used": flow_name,
            }

            self._save_execution_result(execution_result)

            return execution_result

        except Exception as e:
            error_result = {
                "task_id": task_id,
                "status": "error",
                "error": str(e),
                "executed_at": datetime.now().isoformat(),
            }

            self._save_execution_result(error_result)
            return error_result

    def _simulate_execution(
        self, task_info: Dict[str, Any], inputs: Optional[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Simulación de ejecución cuando LangFlow no está disponible."""
        goal = task_info["goal"]

        # Simulación básica de descomposición
        simulated_steps = [
            {"step": 1, "action": "Analyze goal", "description": f"Analyzing: {goal}"},
            {
                "step": 2,
                "action": "Plan execution",
                "description": "Creating execution plan",
            },
            {
                "step": 3,
                "action": "Execute tasks",
                "description": "Executing planned tasks",
            },
            {
                "step": 4,
                "action": "Review results",
                "description": "Reviewing and validating results",
            },
        ]

        simulated_results = {
            "goal": goal,
            "steps_completed": len(simulated_steps),
            "success_rate": 0.85,
            "execution_time": 2.5,
            "outputs": [
                f"Simulated output for step {i}"
                for i in range(1, len(simulated_steps) + 1)
            ],
        }

        return {
            "task_id": task_info["task_id"],
            "status": "completed",
            "simulation_mode": True,
            "results": simulated_results,
            "executed_at": datetime.now().isoformat(),
        }

    def _select_flow_for_goal(self, goal: str) -> str:
        """Seleccionar flow apropiado basado en el objetivo."""
        goal_lower = goal.lower()

        # Heurísticas simples para selección de flow
        if any(
            keyword in goal_lower
            for keyword in ["code", "program", "develop", "implement"]
        ):
            return "development_flow"
        elif any(
            keyword in goal_lower for keyword in ["analyze", "investigate", "research"]
        ):
            return "analysis_flow"
        elif any(keyword in goal_lower for keyword in ["create", "build", "design"]):
            return "creative_flow"
        elif any(
            keyword in goal_lower for keyword in ["fix", "repair", "debug", "solve"]
        ):
            return "problem_solving_flow"
        else:
            return "general_flow"

    def _get_memory_context(
        self, goal: str, memory_types: Optional[List[str]]
    ) -> Dict[str, Any]:
        """Obtener contexto relevante desde Unified Memory Cortex."""
        try:
            from modules.humanoid.cortex.unified_memory import \
                get_unified_memory

            memory = get_unified_memory()
            memories = memory.recall(goal, memory_types=memory_types, limit=10)

            return {
                "relevant_memories": [m.to_dict() for m in memories],
                "memory_sources": list(set(m.source for m in memories)),
                "total_found": len(memories),
            }
        except Exception as e:
            print(f"⚠️ Error obteniendo contexto de memoria: {e}")
            return {}

    def _save_task_chain(self, chain_info: Dict[str, Any]):
        """Guardar información de la cadena en memoria."""
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            chroma.add_memory(
                content=json.dumps(chain_info, default=str),
                memory_type="task_chain",
                metadata={
                    "task_id": chain_info["task_id"],
                    "goal": chain_info["goal"],
                    "flow_name": chain_info["flow_name"],
                    "status": chain_info["status"],
                },
            )
        except Exception as e:
            print(f"⚠️ Error guardando task chain: {e}")

    def _save_execution_result(self, result: Dict[str, Any]):
        """Guardar resultado de ejecución en memoria."""
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            chroma.add_memory(
                content=json.dumps(result, default=str),
                memory_type="execution_result",
                metadata={
                    "task_id": result["task_id"],
                    "status": result["status"],
                    "executed_at": result.get("executed_at"),
                },
            )
        except Exception as e:
            print(f"⚠️ Error guardando execution result: {e}")

    def _get_task_chain(self, task_id: str) -> Optional[Dict[str, Any]]:
        """Recuperar información de cadena desde memoria."""
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            results = chroma.search_memories(
                query=task_id, memory_types=["task_chain"], limit=1
            )

            if results:
                return json.loads(results[0]["content"])
        except Exception:
            pass

        return None

    def list_task_chains(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Listar cadenas de tareas recientes."""
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            results = chroma.search_memories(
                query="task_chain", memory_types=["task_chain"], limit=limit
            )

            chains = []
            for result in results:
                try:
                    chain_info = json.loads(result["content"])
                    chains.append(chain_info)
                except:
                    continue

            return chains
        except Exception:
            return []

    def get_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del orquestador."""
        chains = self.list_task_chains(limit=100)

        status_counts = {}
        flow_counts = {}

        for chain in chains:
            status = chain.get("status", "unknown")
            flow = chain.get("flow_name", "unknown")

            status_counts[status] = status_counts.get(status, 0) + 1
            flow_counts[flow] = flow_counts.get(flow, 0) + 1

        return {
            "total_chains": len(chains),
            "loaded_flows": len(self.loaded_flows),
            "status_distribution": status_counts,
            "flow_distribution": flow_counts,
            "memory_enabled": self.memory_enabled,
            "langflow_available": self.langflow_available,
        }


# Instancia global
_instance: Optional[LangFlowOrchestrator] = None


def get_langflow_orchestrator() -> LangFlowOrchestrator:
    """Obtener instancia global del orquestador LangFlow."""
    global _instance
    if _instance is None:
        _instance = LangFlowOrchestrator()
    return _instance


if __name__ == "__main__":
    # Test básico
    orchestrator = LangFlowOrchestrator()

    # Crear cadena de tareas
    chain = orchestrator.create_task_chain(
        goal="Implementar sistema de memoria con ChromaDB",
        context={"priority": "high", "domain": "development"},
    )

    print(f"Chain creada: {chain['task_id']}")

    # Ejecutar
    result = orchestrator.execute_task_chain(chain["task_id"])
    print(f"Ejecución: {result['status']}")

    # Estadísticas
    stats = orchestrator.get_statistics()
    print(f"Estadísticas: {stats}")
