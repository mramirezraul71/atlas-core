"""
DecisionMaker: Tomador de decisiones del área premotora.

Análogo biológico: Corteza premotora + corteza cingulada anterior
- Evaluación de opciones con función de utilidad
- Balance riesgo/beneficio
- Adaptación según estado interno
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class ActionOption:
    """Opción de acción a evaluar."""
    id: str
    action_type: str
    description: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    estimated_duration_s: float = 5.0
    estimated_success_prob: float = 0.8
    estimated_risk: float = 0.1
    prerequisites: List[str] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "action_type": self.action_type,
            "description": self.description,
            "parameters": self.parameters,
            "estimated_duration_s": self.estimated_duration_s,
            "estimated_success_prob": self.estimated_success_prob,
            "estimated_risk": self.estimated_risk,
        }


@dataclass
class Context:
    """Contexto para toma de decisiones."""
    world_state: Any = None
    internal_state: Dict[str, float] = field(default_factory=dict)
    active_goal: Optional[str] = None
    urgency: float = 0.5  # 0-1
    mode: str = "normal"  # normal, safety_focus, efficiency_focus, learning
    
    @property
    def is_safety_focus(self) -> bool:
        return self.mode == "safety_focus" or self.internal_state.get("safety_priority", 0) > 0.7
    
    @property
    def battery_low(self) -> bool:
        return self.internal_state.get("battery", 100) < 20
    
    @property
    def human_nearby(self) -> bool:
        return self.internal_state.get("human_distance", float("inf")) < 1.5


@dataclass
class Decision:
    """Decisión tomada."""
    action: ActionOption
    confidence: float
    reasoning: str
    alternatives: List[ActionOption] = field(default_factory=list)
    risk_assessment: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action.to_dict(),
            "confidence": self.confidence,
            "reasoning": self.reasoning,
            "alternatives": [a.to_dict() for a in self.alternatives],
            "risk_assessment": self.risk_assessment,
        }


class UtilityFunction:
    """
    Función de utilidad multi-objetivo.
    
    Evalúa acciones considerando:
    - Probabilidad de éxito
    - Alineación con objetivo
    - Eficiencia (tiempo/energía)
    - Preferencias aprendidas
    """
    
    def __init__(self):
        # Pesos de factores (ajustables)
        self.weights = {
            "success_prob": 1.0,
            "goal_alignment": 1.5,
            "efficiency": 0.8,
            "novelty": 0.2,  # Exploración
            "learned_preference": 0.5,
        }
        
        # Preferencias aprendidas por tipo de acción
        self.learned_preferences: Dict[str, float] = {}
    
    def evaluate(self, option: ActionOption, context: Context) -> float:
        """
        Evalúa utilidad de una opción.
        
        Args:
            option: Opción a evaluar
            context: Contexto de decisión
        
        Returns:
            Score de utilidad (mayor = mejor)
        """
        scores = {}
        
        # Probabilidad de éxito
        scores["success_prob"] = option.estimated_success_prob
        
        # Alineación con objetivo
        scores["goal_alignment"] = self._compute_goal_alignment(option, context)
        
        # Eficiencia (inversamente proporcional a duración)
        max_duration = 60.0  # Normalización
        scores["efficiency"] = 1.0 - min(option.estimated_duration_s / max_duration, 1.0)
        
        # Novedad (para exploración)
        scores["novelty"] = 0.5  # Base, podría ajustarse según historial
        
        # Preferencia aprendida
        pref_key = f"{option.action_type}:{option.description[:20]}"
        scores["learned_preference"] = self.learned_preferences.get(pref_key, 0.5)
        
        # Calcular utilidad ponderada
        utility = sum(
            self.weights[factor] * scores.get(factor, 0)
            for factor in self.weights
        )
        
        # Normalizar
        total_weight = sum(self.weights.values())
        utility /= total_weight
        
        return utility
    
    def _compute_goal_alignment(self, option: ActionOption, context: Context) -> float:
        """Calcula alineación de acción con objetivo actual."""
        if not context.active_goal:
            return 0.5  # Neutral
        
        goal = context.active_goal.lower()
        action_desc = option.description.lower()
        
        # Heurística simple: palabras compartidas
        goal_words = set(goal.split())
        desc_words = set(action_desc.split())
        
        if not goal_words:
            return 0.5
        
        overlap = len(goal_words & desc_words)
        return min(overlap / len(goal_words), 1.0)
    
    def update_preference(self, action_type: str, description: str, reward: float) -> None:
        """Actualiza preferencia aprendida basada en resultado."""
        pref_key = f"{action_type}:{description[:20]}"
        current = self.learned_preferences.get(pref_key, 0.5)
        # Running average
        self.learned_preferences[pref_key] = 0.9 * current + 0.1 * reward


class RiskEvaluator:
    """
    Evaluador de riesgo de acciones.
    
    Considera:
    - Riesgo inherente de la acción
    - Contexto (humanos cerca, batería baja, etc.)
    - Historial de fallos
    """
    
    # Riesgo base por tipo de acción
    BASE_RISKS = {
        "navigate": 0.1,
        "grasp": 0.2,
        "place": 0.15,
        "look": 0.02,
        "speak": 0.01,
        "wait": 0.0,
        "execute": 0.3,
    }
    
    def __init__(self):
        self.failure_history: Dict[str, int] = {}
    
    def evaluate(self, option: ActionOption, context: Context) -> float:
        """
        Evalúa riesgo de una opción.
        
        Args:
            option: Opción a evaluar
            context: Contexto de decisión
        
        Returns:
            Score de riesgo (0-1, mayor = más riesgoso)
        """
        # Riesgo base
        base_risk = self.BASE_RISKS.get(option.action_type, 0.2)
        
        # Riesgo estimado por la opción
        estimated_risk = option.estimated_risk
        
        # Factores contextuales
        context_multiplier = 1.0
        
        if context.human_nearby:
            context_multiplier *= 1.5  # Más cuidado cerca de humanos
        
        if context.battery_low:
            context_multiplier *= 1.2  # Más conservador con batería baja
        
        if context.urgency > 0.8:
            context_multiplier *= 0.8  # Acepta más riesgo si es urgente
        
        # Historial de fallos
        failure_key = f"{option.action_type}:{option.description[:20]}"
        failures = self.failure_history.get(failure_key, 0)
        history_factor = min(failures * 0.1, 0.3)  # Hasta 30% extra por historial
        
        # Combinar
        total_risk = (base_risk + estimated_risk) / 2 * context_multiplier + history_factor
        
        return min(total_risk, 1.0)
    
    def record_failure(self, action_type: str, description: str) -> None:
        """Registra fallo de acción."""
        key = f"{action_type}:{description[:20]}"
        self.failure_history[key] = self.failure_history.get(key, 0) + 1
    
    def record_success(self, action_type: str, description: str) -> None:
        """Registra éxito de acción (reduce contador de fallos)."""
        key = f"{action_type}:{description[:20]}"
        if key in self.failure_history:
            self.failure_history[key] = max(0, self.failure_history[key] - 1)


class DecisionMaker:
    """
    Tomador de decisiones del lóbulo frontal.
    
    Evalúa opciones de acción y selecciona la mejor considerando:
    - Utilidad esperada
    - Riesgo
    - Estado interno (modo de operación)
    """
    
    def __init__(self):
        self.utility_function = UtilityFunction()
        self.risk_evaluator = RiskEvaluator()
        
        # Umbrales de decisión
        self.min_confidence = 0.3
        self.max_acceptable_risk = 0.7
    
    def decide(self, options: List[ActionOption], context: Context) -> Decision:
        """
        Decide entre opciones de acción.
        
        Args:
            options: Lista de opciones a considerar
            context: Contexto de decisión
        
        Returns:
            Decision con acción seleccionada
        """
        if not options:
            raise ValueError("No options provided for decision")
        
        scored_options = []
        
        for option in options:
            utility = self.utility_function.evaluate(option, context)
            risk = self.risk_evaluator.evaluate(option, context)
            
            # Balanceo utilidad/riesgo según modo
            if context.is_safety_focus:
                # Penalizar fuertemente el riesgo
                score = utility - 2.0 * risk
            elif context.mode == "efficiency_focus":
                # Aceptar más riesgo por eficiencia
                score = utility - 0.3 * risk
            elif context.mode == "learning":
                # Favorecer exploración
                score = utility - 0.5 * risk + 0.2  # Bonus exploración
            else:
                # Normal
                score = utility - 0.5 * risk
            
            scored_options.append({
                "option": option,
                "utility": utility,
                "risk": risk,
                "score": score,
            })
        
        # Ordenar por score
        scored_options.sort(key=lambda x: x["score"], reverse=True)
        
        # Seleccionar mejor opción
        best = scored_options[0]
        
        # Verificar umbrales
        if best["risk"] > self.max_acceptable_risk and not context.urgency > 0.9:
            # Riesgo muy alto, reconsiderar
            logger.warning(f"High risk action selected: {best['option'].description}, risk={best['risk']:.2f}")
        
        # Construir razonamiento
        reasoning = self._build_reasoning(best, context)
        
        # Alternativas (top 3)
        alternatives = [s["option"] for s in scored_options[1:4]]
        
        return Decision(
            action=best["option"],
            confidence=min(best["score"], 1.0),
            reasoning=reasoning,
            alternatives=alternatives,
            risk_assessment=best["risk"],
        )
    
    def _build_reasoning(self, scored: Dict, context: Context) -> str:
        """Construye explicación de la decisión."""
        option = scored["option"]
        parts = [
            f"Seleccionada: {option.description}",
            f"Utilidad: {scored['utility']:.2f}",
            f"Riesgo: {scored['risk']:.2f}",
            f"Score final: {scored['score']:.2f}",
        ]
        
        if context.is_safety_focus:
            parts.append("Modo: enfoque en seguridad")
        
        if context.human_nearby:
            parts.append("Contexto: humano cercano")
        
        return " | ".join(parts)
    
    def feedback(self, action: ActionOption, success: bool, reward: float = None) -> None:
        """
        Proporciona feedback sobre resultado de acción.
        
        Args:
            action: Acción ejecutada
            success: Si fue exitosa
            reward: Recompensa opcional (0-1)
        """
        if success:
            self.risk_evaluator.record_success(action.action_type, action.description)
            self.utility_function.update_preference(
                action.action_type, action.description, 
                reward if reward is not None else 0.8
            )
        else:
            self.risk_evaluator.record_failure(action.action_type, action.description)
            self.utility_function.update_preference(
                action.action_type, action.description,
                reward if reward is not None else 0.2
            )
