"""
Agent Cycle Prevention - Prevención de ciclos infinitos en el agente
Implementa interruptores automáticos y lógica de escape
"""
from __future__ import annotations

import logging
import time
from collections import defaultdict, deque
from typing import Any, Dict, Optional

_log = logging.getLogger("atlas.cycle_prevention")

# Configuración de prevención de ciclos
CYCLE_PREVENTION_CONFIG = {
    "max_iterations": 50,  # Límite estricto de iteraciones
    "tool_repetition_limit": 3,  # Máximo de repetición del mismo tool
    "file_repetition_limit": 2,  # Máximo de acceso al mismo archivo
    "command_repetition_limit": 2,  # Máximo del mismo comando
    "progress_stagnation_limit": 10,  # Iteraciones sin progreso antes de escape (aumentado de 5)
    "memory_window": 10,  # Ventana de memoria para análisis
    "escape_strategies": [
        "summarize_and_continue",
        "change_approach",
        "ask_for_clarification",
        "declare_completion",
    ],
}


class AgentCyclePrevention:
    """
    Sistema de prevención de ciclos infinitos en el agente Atlas.

    Implementa múltiples estrategias de escape y detección temprana
    para evitar que el agente se quede atascado.
    """

    def __init__(self, config: Optional[Dict] = None):
        self.config = config or CYCLE_PREVENTION_CONFIG

        # Estado de seguimiento
        self.iteration_count = 0
        self.tool_history: deque = deque(maxlen=self.config["memory_window"])
        self.file_history: deque = deque(maxlen=self.config["memory_window"])
        self.command_history: deque = deque(maxlen=self.config["memory_window"])
        self.progress_history: deque = deque(maxlen=self.config["memory_window"])

        # Contadores de repetición
        self.tool_repetition_counter: Dict[str, int] = defaultdict(int)
        self.file_repetition_counter: Dict[str, int] = defaultdict(int)
        self.command_repetition_counter: Dict[str, int] = defaultdict(int)

        # Estado de ciclo
        self.cycle_detected = False
        self.cycle_start_iteration = None
        self.escape_attempts = 0

        _log.info("Agent Cycle Prevention initialized")

    def check_iteration_limits(self) -> Dict[str, Any]:
        """
        Verifica si se ha alcanzado algún límite de ciclo.

        Returns:
            Resultado de la verificación
        """
        self.iteration_count += 1

        check_result = {
            "iteration": self.iteration_count,
            "should_escape": False,
            "escape_reason": None,
            "escape_strategy": None,
            "warnings": [],
        }

        # 1. Límite máximo de iteraciones
        if self.iteration_count >= self.config["max_iterations"]:
            check_result["should_escape"] = True
            check_result[
                "escape_reason"
            ] = f"Máximo de iteraciones alcanzado ({self.config['max_iterations']})"
            check_result["escape_strategy"] = "declare_completion"
            check_result["warnings"].append("Límite de iteraciones alcanzado")

        # 2. Repetición de tools
        if self._check_tool_repetition(check_result):
            check_result["should_escape"] = True
            check_result["warnings"].append("Repetición de tools detectada")

        # 3. Repetición de archivos
        if self._check_file_repetition(check_result):
            check_result["should_escape"] = True
            check_result["warnings"].append("Repetición de archivos detectada")

        # 4. Repetición de comandos
        if self._check_command_repetition(check_result):
            check_result["should_escape"] = True
            check_result["warnings"].append("Repetición de comandos detectada")

        # 5. Estancamiento de progreso
        if self._check_progress_stagnation(check_result):
            check_result["should_escape"] = True
            check_result["warnings"].append("Estancamiento de progreso detectado")

        # 6. Patrones de ciclo complejos
        if self._check_complex_patterns(check_result):
            check_result["should_escape"] = True
            check_result["warnings"].append("Patrón de ciclo complejo detectado")

        # Si se detecta ciclo, registrar inicio
        if check_result["should_escape"] and not self.cycle_detected:
            self.cycle_detected = True
            self.cycle_start_iteration = self.iteration_count
            _log.warning(
                f"Ciclo detectado en iteración {self.iteration_count}: {check_result['escape_reason']}"
            )

        return check_result

    def record_tool_call(self, tool_name: str, tool_input: Dict) -> None:
        """Registra una llamada a tool para análisis de ciclo."""
        # Use a more specific key for certain tools to avoid false-positive "repetition"
        # when the tool is used normally against different targets (e.g., atlas_api to different endpoints).
        tool_key = tool_name
        if tool_name == "atlas_api":
            try:
                method = str((tool_input or {}).get("method") or "").upper() or "GET"
                endpoint = str((tool_input or {}).get("endpoint") or "").strip() or "?"
                tool_key = f"atlas_api:{method}:{endpoint}"
            except Exception:
                tool_key = tool_name
        elif tool_name == "list_directory":
            try:
                path = str((tool_input or {}).get("path") or "").strip() or "?"
                tool_key = f"list_directory:{path}"
            except Exception:
                tool_key = tool_name
        elif tool_name == "read_file":
            try:
                path = (
                    str(
                        (tool_input or {}).get("path")
                        or (tool_input or {}).get("file_path")
                        or ""
                    ).strip()
                    or "?"
                )
                tool_key = f"read_file:{path}"
            except Exception:
                tool_key = tool_name
        elif tool_name == "search_text":
            try:
                patt = str((tool_input or {}).get("pattern") or "").strip() or "?"
                directory = (
                    str((tool_input or {}).get("directory") or "").strip() or "?"
                )
                tool_key = f"search_text:{directory}:{patt[:80]}"
            except Exception:
                tool_key = tool_name
        tool_record = {
            "name": tool_key,
            "input": tool_input,
            "timestamp": time.time(),
            "iteration": self.iteration_count,
        }

        self.tool_history.append(tool_record)
        self.tool_repetition_counter[tool_key] += 1

        # Extraer archivos y comandos del input
        self._extract_files_and_commands(tool_name, tool_input)

    def record_progress(self, progress_pct: int) -> None:
        """Registra el progreso para detectar estancamiento."""
        self.progress_history.append(
            {
                "progress": progress_pct,
                "timestamp": time.time(),
                "iteration": self.iteration_count,
            }
        )

    def _extract_files_and_commands(self, tool_name: str, tool_input: Dict) -> None:
        """Extrae archivos y comandos del input del tool."""
        # Detectar archivos
        if tool_name in ["read_file", "write_to_file", "edit_file"]:
            file_path = tool_input.get("file_path", "")
            if file_path:
                self.file_history.append(file_path)
                self.file_repetition_counter[file_path] += 1

        # Detectar comandos
        if tool_name == "execute_command":
            command = tool_input.get("CommandLine", "")
            if command:
                self.command_history.append(command)
                self.command_repetition_counter[command] += 1

    def _check_tool_repetition(self, check_result: Dict) -> bool:
        """Verifica repetición de tools."""
        default_limit = int(self.config.get("tool_repetition_limit", 3) or 3)
        for tool_name, count in self.tool_repetition_counter.items():
            # More tolerant thresholds for exploratory tools (common in healthy runs).
            if tool_name.startswith("list_directory:"):
                limit = 6
            elif tool_name.startswith("read_file:"):
                limit = 6
            elif tool_name.startswith("search_text:"):
                limit = 6
            elif tool_name.startswith("atlas_api:"):
                limit = 6
            else:
                limit = default_limit
            if count >= limit:
                check_result[
                    "escape_reason"
                ] = f"Tool '{tool_name}' repetido {count} veces"
                check_result["escape_strategy"] = "change_approach"
                return True
        return False

    def _check_file_repetition(self, check_result: Dict) -> bool:
        """Verifica repetición de archivos."""
        for file_path, count in self.file_repetition_counter.items():
            if count >= self.config["file_repetition_limit"]:
                check_result[
                    "escape_reason"
                ] = f"Archivo '{file_path}' accedido {count} veces"
                check_result["escape_strategy"] = "change_approach"
                return True
        return False

    def _check_command_repetition(self, check_result: Dict) -> bool:
        """Verifica repetición de comandos."""
        for command, count in self.command_repetition_counter.items():
            if count >= self.config["command_repetition_limit"]:
                check_result[
                    "escape_reason"
                ] = f"Comando repetido {count} veces: {command[:50]}"
                check_result["escape_strategy"] = "change_approach"
                return True
        return False

    def _check_progress_stagnation(self, check_result: Dict) -> bool:
        """Verifica estancamiento de progreso."""
        if len(self.progress_history) < self.config["progress_stagnation_limit"]:
            return False

        # Verificar últimas N iteraciones
        recent_progress = [
            entry["progress"]
            for entry in list(self.progress_history)[
                -self.config["progress_stagnation_limit"] :
            ]
        ]

        # Si todo el progreso es el mismo, hay estancamiento
        if len(set(recent_progress)) == 1:
            check_result[
                "escape_reason"
            ] = f"Progreso estancado en {recent_progress[0]}% por {self.config['progress_stagnation_limit']} iteraciones"
            check_result["escape_strategy"] = "summarize_and_continue"
            return True

        return False

    def _check_complex_patterns(self, check_result: Dict) -> bool:
        """Verifica patrones complejos de ciclo."""
        # Patrón 1: Mismo tool con mismo input
        if len(self.tool_history) >= 3:
            last_three = list(self.tool_history)[-3:]
            if (
                last_three[0]["name"] == last_three[1]["name"] == last_three[2]["name"]
                and last_three[0]["input"]
                == last_three[1]["input"]
                == last_three[2]["input"]
            ):
                repeated_name = str(last_three[0]["name"] or "")
                # Exploratory tools may legitimately repeat with same input in short bursts.
                if not (
                    repeated_name.startswith("list_directory:")
                    or repeated_name.startswith("read_file:")
                    or repeated_name.startswith("search_text:")
                    or repeated_name.startswith("atlas_api:")
                ):
                    check_result[
                        "escape_reason"
                    ] = f"Tool repetido con mismo input: {repeated_name}"
                    check_result["escape_strategy"] = "ask_for_clarification"
                    return True

        # Patrón 2: Secuencia circular de tools
        if len(self.tool_history) >= 4:
            recent_tools = [t["name"] for t in list(self.tool_history)[-4:]]
            if len(set(recent_tools)) == 2 and recent_tools[0] == recent_tools[2]:
                check_result[
                    "escape_reason"
                ] = f"Secuencia circular detectada: {recent_tools}"
                check_result["escape_strategy"] = "change_approach"
                return True

        return False

    def generate_escape_message(self, escape_reason: str, strategy: str) -> str:
        """
        Genera un mensaje de escape para el LLM.

        Args:
            escape_reason: Razón del escape
            strategy: Estrategia de escape

        Returns:
            Mensaje formateado para el LLM
        """
        base_message = f"⚠️ DETENCIÓN AUTOMÁTICA: {escape_reason}\n\n"

        if strategy == "summarize_and_continue":
            base_message += """Por favor, resume lo que has hecho hasta ahora y continúa con un enfoque diferente.
Evita repetir las mismas acciones que ya has intentado."""

        elif strategy == "change_approach":
            base_message += """Por favor, cambia completamente tu enfoque.
Intenta una estrategia diferente o herramientas distintas."""

        elif strategy == "ask_for_clarification":
            base_message += """Parece que estás atascado. Por favor, pide clarificación sobre el objetivo o sugiere un plan diferente."""

        elif strategy == "declare_completion":
            base_message += """Has alcanzado el límite de iteraciones. Por favor, resume lo que has logrado y declara si la tarea está completa o necesita un enfoque diferente."""

        # Añadir contexto del estado actual
        base_message += f"\n\nEstado actual:\n- Iteración: {self.iteration_count}\n- Tools usados: {len(self.tool_history)}\n- Últimos tools: {[t['name'] for t in list(self.tool_history)[-3:]]}"

        return base_message

    def reset(self) -> None:
        """Reinicia el estado del detector de ciclos."""
        self.iteration_count = 0
        self.tool_history.clear()
        self.file_history.clear()
        self.command_history.clear()
        self.progress_history.clear()
        self.tool_repetition_counter.clear()
        self.file_repetition_counter.clear()
        self.command_repetition_counter.clear()
        self.cycle_detected = False
        self.cycle_start_iteration = None
        self.escape_attempts = 0

        _log.info("Cycle Prevention reset")

    def get_status(self) -> Dict[str, Any]:
        """Obtener el estado actual del sistema de prevención."""
        return {
            "iteration_count": self.iteration_count,
            "cycle_detected": self.cycle_detected,
            "cycle_start_iteration": self.cycle_start_iteration,
            "escape_attempts": self.escape_attempts,
            "tool_history_size": len(self.tool_history),
            "unique_tools": len(set(t["name"] for t in self.tool_history)),
            "repeated_tools": len(
                [t for t, c in self.tool_repetition_counter.items() if c > 1]
            ),
            "config": self.config,
        }


# Instancia global para uso en el agent engine
_cycle_prevention: Optional[AgentCyclePrevention] = None


def get_cycle_prevention() -> AgentCyclePrevention:
    """Obtener instancia global del sistema de prevención de ciclos."""
    global _cycle_prevention
    if _cycle_prevention is None:
        _cycle_prevention = AgentCyclePrevention()
    return _cycle_prevention


def integrate_cycle_prevention(agent_engine_func):
    """
    Decorador para integrar prevención de ciclos en el agent engine.

    Args:
        agent_engine_func: Función original del agent engine

    Returns:
        Función con prevención de ciclos integrada
    """

    def wrapped_agent_engine(*args, **kwargs):
        prevention = get_cycle_prevention()
        prevention.reset()  # Reset para nueva tarea

        # Iterar sobre el generador original con prevención
        for event in agent_engine_func(*args, **kwargs):
            # Si es un tool call, registrar para análisis
            if event.get("event") == "tool_call":
                tool_data = event.get("data", {})
                prevention.record_tool_call(
                    tool_data.get("name", ""), tool_data.get("input", {})
                )

            # Si es progreso, registrar
            if event.get("event") == "tool_result":
                progress = event.get("data", {}).get("progress_pct", 0)
                prevention.record_progress(progress)

            # Verificar límites de ciclo
            cycle_check = prevention.check_iteration_limits()

            if cycle_check["should_escape"]:
                # Generar evento de escape
                escape_message = prevention.generate_escape_message(
                    cycle_check["escape_reason"], cycle_check["escape_strategy"]
                )

                yield {
                    "event": "cycle_detected",
                    "data": {
                        "reason": cycle_check["escape_reason"],
                        "strategy": cycle_check["escape_strategy"],
                        "iteration": cycle_check["iteration"],
                        "warnings": cycle_check["warnings"],
                        "escape_message": escape_message,
                    },
                }

                # Inyectar mensaje de escape en el flujo
                yield {
                    "event": "text",
                    "data": {"content": escape_message, "llm_ms": 0},
                }

                yield {
                    "event": "done",
                    "data": {
                        "iterations": cycle_check["iteration"],
                        "cycle_escaped": True,
                        "escape_reason": cycle_check["escape_reason"],
                        "escape_strategy": cycle_check["escape_strategy"],
                    },
                }

                break

            # Continuar con el evento original
            yield event

    return wrapped_agent_engine


if __name__ == "__main__":
    # Test del sistema de prevención
    prevention = AgentCyclePrevention()

    print("Test de detección de ciclos:")

    # Simular iteraciones normales
    for i in range(1, 15):
        prevention.record_tool_call(f"tool_{i % 3}", {"param": "value"})
        prevention.record_progress(i * 5)

        result = prevention.check_iteration_limits()
        if result["should_escape"]:
            print(f"Escape en iteración {i}: {result['escape_reason']}")
            break

    # Simular ciclo de tool repetido
    print("\nTest de repetición de tool:")
    for i in range(1, 10):
        prevention.record_tool_call("execute_command", {"CommandLine": "ls -la"})
        result = prevention.check_iteration_limits()
        if result["should_escape"]:
            print(f"Escape por repetición: {result['escape_reason']}")
            break

    # Mostrar estado final
    status = prevention.get_status()
    print(f"\nEstado final: {status}")
