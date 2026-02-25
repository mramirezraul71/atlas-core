"""
Agent Cycle Analysis - Diagnóstico de ciclos infinitos en el agente
Identifica patrones que causan que el agente no termine las tareas
"""
from __future__ import annotations

import json
import logging
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_log = logging.getLogger("atlas.cycle_analysis")

# Patrones que pueden causar ciclos
CYCLE_PATTERNS = {
    "repetitive_tool_calls": [
        r"execute_command.*git status",
        r"execute_command.*ls.*-la",
        r"read_file.*same_file",
        r"execute_command.*cat.*file",
    ],
    "infinite_loops": [
        r"while.*True",
        r"for.*in.*range.*\d+",
        r"recursiv.*sin.*caso.*base",
        r"loop.*infinit",
    ],
    "redundant_analysis": [
        r"analiza.*el.*mismo.*archivo",
        r"revisa.*el.*mismo.*error",
        r"check.*again.*same.*issue",
        r"verify.*same.*thing",
    ],
    "tool_chain_repetition": [
        r"execute_command.*cd.*directory",
        r"execute_command.*pwd",
        r"execute_command.*echo.*path",
        r"list_directory.*same.*path",
    ],
    "model_confusion": [
        r"try.*different.*model",
        r"switch.*provider",
        r"failover.*again",
        r"retry.*same.*command",
    ],
}

# Señales de ciclo detectado
CYCLE_INDICATORS = {
    "high_iteration_count": 50,  # Más de 50 iteraciones es sospechoso
    "tool_repetition_threshold": 5,  # Mismo tool 5 veces seguidas
    "file_repetition_threshold": 3,  # Mismo archivo 3 veces seguidas
    "command_repetition_threshold": 3,  # Mismo comando 3 veces seguidas
    "progress_stagnation": 10,  # Sin progreso por 10 iteraciones
}


class AgentCycleDetector:
    """
    Detector de ciclos infinitos en el agente Atlas.

    Analiza el flujo de ejecución para identificar patrones problemáticos
    que causan que el agente no termine las tareas.
    """

    def __init__(self):
        self.iteration_history: List[Dict] = []
        self.tool_calls_history: List[Dict] = []
        self.current_cycle_start: Optional[int] = None
        self.detected_cycles: List[Dict] = []

    def analyze_iteration(
        self,
        iteration: int,
        tool_calls: List[Dict],
        messages: List[Dict],
        progress_pct: int,
    ) -> Dict[str, Any]:
        """
        Analiza una iteración en busca de patrones de ciclo.

        Args:
            iteration: Número de iteración actual
            tool_calls: Tools llamados en esta iteración
            messages: Mensajes del LLM
            progress_pct: Porcentaje de progreso

        Returns:
            Análisis de ciclo detectado
        """
        analysis = {
            "iteration": iteration,
            "timestamp": time.time(),
            "tool_count": len(tool_calls),
            "progress_pct": progress_pct,
            "cycle_indicators": [],
            "risk_level": "low",
            "recommendations": [],
        }

        # 1. Verificar conteo alto de iteraciones
        if iteration > CYCLE_INDICATORS["high_iteration_count"]:
            analysis["cycle_indicators"].append("high_iteration_count")
            analysis["risk_level"] = "high"
            analysis["recommendations"].append(
                "Considerar límite de iteraciones más bajo"
            )

        # 2. Detectar repetición de tools
        tool_repetition = self._detect_tool_repetition(tool_calls)
        if tool_repetition:
            analysis["cycle_indicators"].append("tool_repetition")
            analysis["tool_repetition"] = tool_repetition
            if (
                tool_repetition["count"]
                >= CYCLE_INDICATORS["tool_repetition_threshold"]
            ):
                analysis["risk_level"] = "high"
                analysis["recommendations"].append(
                    f"Tool '{tool_repetition['tool']}' repetido {tool_repetition['count']} veces"
                )

        # 3. Detectar estancamiento de progreso
        progress_stagnation = self._detect_progress_stagnation(progress_pct)
        if progress_stagnation:
            analysis["cycle_indicators"].append("progress_stagnation")
            analysis["progress_stagnation"] = progress_stagnation
            if (
                progress_stagnation["stagnant_iterations"]
                >= CYCLE_INDICATORS["progress_stagnation"]
            ):
                analysis["risk_level"] = "medium"
                analysis["recommendations"].append("Progreso estancado, posible ciclo")

        # 4. Analizar patrones en mensajes
        message_patterns = self._analyze_message_patterns(messages)
        if message_patterns:
            analysis["cycle_indicators"].extend(message_patterns["patterns"])
            analysis["message_patterns"] = message_patterns
            if "repetitive_analysis" in message_patterns["patterns"]:
                analysis["risk_level"] = "medium"
                analysis["recommendations"].append("Análisis redundante detectado")

        # 5. Verificar si estamos en un ciclo activo
        if self.current_cycle_start is None and analysis["risk_level"] in [
            "medium",
            "high",
        ]:
            self.current_cycle_start = iteration
            analysis["cycle_started"] = True
        elif self.current_cycle_start and analysis["risk_level"] == "low":
            cycle_duration = iteration - self.current_cycle_start
            self.detected_cycles.append(
                {
                    "start_iteration": self.current_cycle_start,
                    "end_iteration": iteration,
                    "duration": cycle_duration,
                    "indicators": analysis["cycle_indicators"],
                }
            )
            self.current_cycle_start = None
            analysis["cycle_ended"] = True
            analysis["cycle_duration"] = cycle_duration

        # Guardar en historial
        self.iteration_history.append(analysis)
        self.tool_calls_history.extend(
            [
                {
                    "iteration": iteration,
                    "tool": tool.get("name"),
                    "input": tool.get("input"),
                }
                for tool in tool_calls
            ]
        )

        return analysis

    def _detect_tool_repetition(self, tool_calls: List[Dict]) -> Optional[Dict]:
        """Detecta repetición de tools consecutivos."""
        if not tool_calls:
            return None

        # Verificar últimos N tool calls
        recent_tools = [call.get("name") for call in self.tool_calls_history[-10:]]

        if not recent_tools:
            return None

        # Contar repeticiones consecutivas del último tool
        last_tool = tool_calls[0].get("name")
        consecutive_count = 0

        for tool in reversed(recent_tools):
            if tool == last_tool:
                consecutive_count += 1
            else:
                break

        if consecutive_count >= CYCLE_INDICATORS["tool_repetition_threshold"]:
            return {
                "tool": last_tool,
                "count": consecutive_count,
                "pattern": "consecutive_repetition",
            }

        return None

    def _detect_progress_stagnation(self, current_progress: int) -> Optional[Dict]:
        """Detecta estancamiento en el progreso."""
        if len(self.iteration_history) < 5:
            return None

        # Verificar últimas iteraciones
        recent_progress = [
            analysis.get("progress_pct", 0) for analysis in self.iteration_history[-10:]
        ]

        if len(recent_progress) < 5:
            return None

        # Verificar si el progreso no ha cambiado
        stagnant_count = 0
        for i in range(1, len(recent_progress)):
            if recent_progress[i] == recent_progress[i - 1]:
                stagnant_count += 1
            else:
                break

        if stagnant_count >= CYCLE_INDICATORS["progress_stagnation"]:
            return {
                "stagnant_iterations": stagnant_count,
                "stuck_progress": recent_progress[0],
                "pattern": "progress_stagnation",
            }

        return None

    def _analyze_message_patterns(self, messages: List[Dict]) -> Optional[Dict]:
        """Analiza patrones en mensajes del LLM."""
        if not messages:
            return None

        patterns_found = []
        combined_text = ""

        for msg in messages:
            if isinstance(msg, dict) and "text" in msg:
                combined_text += msg["text"] + " "
            elif isinstance(msg, str):
                combined_text += msg + " "

        combined_text = combined_text.lower()

        # Buscar patrones problemáticos
        for pattern_name, pattern_list in CYCLE_PATTERNS.items():
            for pattern in pattern_list:
                if re.search(pattern, combined_text, re.IGNORECASE):
                    patterns_found.append(pattern_name)
                    break

        if patterns_found:
            return {
                "patterns": patterns_found,
                "text_sample": combined_text[:200] + "..."
                if len(combined_text) > 200
                else combined_text,
            }

        return None

    def get_cycle_summary(self) -> Dict[str, Any]:
        """Obtener resumen de ciclos detectados."""
        if not self.iteration_history:
            return {"status": "no_data", "message": "No hay datos para analizar"}

        total_iterations = len(self.iteration_history)
        high_risk_iterations = len(
            [a for a in self.iteration_history if a["risk_level"] == "high"]
        )
        medium_risk_iterations = len(
            [a for a in self.iteration_history if a["risk_level"] == "medium"]
        )

        most_common_indicators = {}
        for analysis in self.iteration_history:
            for indicator in analysis.get("cycle_indicators", []):
                most_common_indicators[indicator] = (
                    most_common_indicators.get(indicator, 0) + 1
                )

        # Ordenar por frecuencia
        sorted_indicators = sorted(
            most_common_indicators.items(), key=lambda x: x[1], reverse=True
        )

        return {
            "status": "analyzed",
            "total_iterations": total_iterations,
            "high_risk_iterations": high_risk_iterations,
            "medium_risk_iterations": medium_risk_iterations,
            "detected_cycles": len(self.detected_cycles),
            "cycles": self.detected_cycles,
            "most_common_indicators": sorted_indicators[:5],
            "current_cycle_active": self.current_cycle_start is not None,
            "recommendations": self._generate_recommendations(),
        }

    def _generate_recommendations(self) -> List[str]:
        """Genera recomendaciones basadas en el análisis."""
        recommendations = []

        if not self.iteration_history:
            return recommendations

        # Analizar patrones comunes sin recursión
        total_iterations = len(self.iteration_history)
        high_risk_iterations = len(
            [a for a in self.iteration_history if a["risk_level"] == "high"]
        )

        # Contar indicadores
        most_common_indicators = {}
        for analysis in self.iteration_history:
            for indicator in analysis.get("cycle_indicators", []):
                most_common_indicators[indicator] = (
                    most_common_indicators.get(indicator, 0) + 1
                )

        # Generar recomendaciones
        if high_risk_iterations > total_iterations * 0.3:
            recommendations.append(
                "Considerar reducir SAFETY_MAX_ITERATIONS de 100 a 50"
            )

        if any("tool_repetition" in ind for ind in most_common_indicators.keys()):
            recommendations.append("Implementar detección de repetición de tools")

        if any("progress_stagnation" in ind for ind in most_common_indicators.keys()):
            recommendations.append(
                "Añadir lógica de escape cuando el progreso se estanca"
            )

        if len(self.detected_cycles) > 0:
            recommendations.append("Implementar interruptor automático de ciclos")

        # Recomendaciones específicas basadas en indicadores
        for indicator, count in most_common_indicators.items():
            if indicator == "high_iteration_count" and count > 5:
                recommendations.append(
                    "Las tareas están requiriendo demasiadas iteraciones"
                )
            elif indicator == "repetitive_analysis" and count > 3:
                recommendations.append(
                    "El LLM está analizando los mismos archivos repetidamente"
                )

        return recommendations


def diagnose_agent_cycles(log_file: Optional[str] = None) -> Dict[str, Any]:
    """
    Diagnóstico completo de ciclos del agente.

    Args:
        log_file: Archivo de log para analizar (opcional)

    Returns:
        Diagnóstico completo
    """
    detector = AgentCycleDetector()

    # Si hay archivo de log, analizarlo
    if log_file and Path(log_file).exists():
        _log.info(f"Analizando log file: {log_file}")
        # TODO: Implementar análisis de archivo de log

    # Simular análisis de iteraciones típicas
    _log.info("Simulando análisis de ciclos del agente...")

    # Generar análisis de ejemplo
    sample_analysis = detector.get_cycle_summary()
    sample_analysis["simulation"] = True
    sample_analysis["analysis_timestamp"] = time.time()

    return sample_analysis


if __name__ == "__main__":
    # Test del detector
    detector = AgentCycleDetector()

    # Simular algunas iteraciones problemáticas
    for i in range(1, 55):
        # Simular tool calls repetitivos
        if i % 10 == 0:
            tool_calls = [
                {"name": "execute_command", "input": {"command": "git status"}}
            ]
        else:
            tool_calls = [{"name": f"tool_{i % 3}", "input": {"param": "value"}}]

        # Simular progreso estancado
        progress = 20 if i % 15 != 0 else 25

        analysis = detector.analyze_iteration(i, tool_calls, [], progress)

        if i % 10 == 0:
            print(
                f"Iteración {i}: {analysis['risk_level']} - {analysis['cycle_indicators']}"
            )

    # Obtener resumen
    summary = detector.get_cycle_summary()
    print(f"\nResumen del análisis:")
    print(f"Iteraciones totales: {summary['total_iterations']}")
    print(f"Ciclos detectados: {summary['detected_cycles']}")
    print(f"Indicadores comunes: {summary['most_common_indicators']}")
    print(f"Recomendaciones: {summary['recommendations']}")
