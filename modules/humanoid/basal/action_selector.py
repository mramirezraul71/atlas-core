"""
ActionSelector: Seleccion de acciones (Ganglios Basales - Estriado).

Implementa el proceso de seleccion entre multiples opciones de accion
basado en utilidad, riesgo y contexto.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class ActionType(str, Enum):
    """Tipos de acciones."""
    MOVE = "move"
    GRASP = "grasp"
    PLACE = "place"
    SPEAK = "speak"
    LOOK = "look"
    WAIT = "wait"
    STOP = "stop"
    GESTURE = "gesture"
    NAVIGATE = "navigate"
    INTERACT = "interact"


class SelectionStrategy(str, Enum):
    """Estrategias de seleccion."""
    GREEDY = "greedy"           # Mejor puntuacion
    SOFTMAX = "softmax"         # Probabilistico
    EPSILON_GREEDY = "epsilon"  # Exploracion/explotacion
    THOMPSON = "thompson"       # Thompson sampling


@dataclass
class ActionCandidate:
    """Candidato a accion."""
    id: str
    action_type: ActionType
    description: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    # Evaluacion
    utility: float = 0.0          # Utilidad esperada
    risk: float = 0.0             # Riesgo asociado
    confidence: float = 1.0       # Confianza en la estimacion
    
    # Metadata
    source: str = "planner"       # Quien propuso la accion
    priority: int = 5             # 1-10, mayor es mas urgente
    deadline_ns: Optional[int] = None
    
    # Resultado de evaluacion
    score: float = 0.0
    blocked: bool = False
    block_reason: Optional[str] = None


@dataclass
class SelectionContext:
    """Contexto para seleccion de acciones."""
    current_goal: Optional[str] = None
    internal_state: Dict[str, float] = field(default_factory=dict)
    world_state: Dict[str, Any] = field(default_factory=dict)
    
    # Modificadores
    safety_priority: float = 1.0   # Factor de seguridad
    speed_priority: float = 1.0    # Factor de velocidad
    explore_rate: float = 0.1      # Tasa de exploracion
    
    # Estado del sistema
    mode: str = "normal"
    
    def is_safety_mode(self) -> bool:
        return self.mode in ("safe", "emergency", "careful")


@dataclass
class SelectionResult:
    """Resultado de la seleccion."""
    selected_action: Optional[ActionCandidate]
    all_candidates: List[ActionCandidate]
    selection_time_ms: float
    strategy_used: SelectionStrategy
    reasoning: str
    alternatives: List[ActionCandidate] = field(default_factory=list)


class ActionSelector:
    """
    Selector de acciones basado en utilidad.
    
    Implementa el proceso de seleccion del estriado:
    - Evalua candidatos
    - Aplica estrategia de seleccion
    - Considera contexto y prioridades
    """
    
    def __init__(
        self,
        strategy: SelectionStrategy = SelectionStrategy.GREEDY,
        risk_weight: float = 1.0,
        utility_weight: float = 1.0,
        confidence_weight: float = 0.5,
    ):
        """
        Inicializa el selector.
        
        Args:
            strategy: Estrategia de seleccion
            risk_weight: Peso del riesgo en la puntuacion
            utility_weight: Peso de la utilidad
            confidence_weight: Peso de la confianza
        """
        self.strategy = strategy
        self.risk_weight = risk_weight
        self.utility_weight = utility_weight
        self.confidence_weight = confidence_weight
        
        # Historial
        self._selection_history: List[SelectionResult] = []
        
        # Evaluadores customizados
        self._utility_evaluators: List[Callable[[ActionCandidate, SelectionContext], float]] = []
        self._risk_evaluators: List[Callable[[ActionCandidate, SelectionContext], float]] = []
        
        # Callbacks
        self._pre_selection_hooks: List[Callable[[List[ActionCandidate]], None]] = []
        self._post_selection_hooks: List[Callable[[SelectionResult], None]] = []
        
        # Estadisticas
        self._total_selections = 0
        self._exploration_count = 0
    
    def select(
        self,
        candidates: List[ActionCandidate],
        context: SelectionContext,
    ) -> SelectionResult:
        """
        Selecciona la mejor accion.
        
        Args:
            candidates: Lista de acciones candidatas
            context: Contexto de seleccion
        
        Returns:
            Resultado con accion seleccionada
        """
        start_time = time.time()
        
        if not candidates:
            return SelectionResult(
                selected_action=None,
                all_candidates=[],
                selection_time_ms=0,
                strategy_used=self.strategy,
                reasoning="No hay candidatos disponibles",
            )
        
        # Pre-hooks
        for hook in self._pre_selection_hooks:
            try:
                hook(candidates)
            except Exception as e:
                logger.error(f"Error in pre-selection hook: {e}")
        
        # Evaluar candidatos
        scored_candidates = self._evaluate_candidates(candidates, context)
        
        # Filtrar bloqueados
        valid_candidates = [c for c in scored_candidates if not c.blocked]
        
        if not valid_candidates:
            return SelectionResult(
                selected_action=None,
                all_candidates=scored_candidates,
                selection_time_ms=(time.time() - start_time) * 1000,
                strategy_used=self.strategy,
                reasoning="Todos los candidatos estan bloqueados",
            )
        
        # Aplicar estrategia
        selected, reasoning = self._apply_strategy(valid_candidates, context)
        
        # Obtener alternativas
        alternatives = [c for c in valid_candidates if c.id != selected.id][:3]
        
        # Resultado
        result = SelectionResult(
            selected_action=selected,
            all_candidates=scored_candidates,
            selection_time_ms=(time.time() - start_time) * 1000,
            strategy_used=self.strategy,
            reasoning=reasoning,
            alternatives=alternatives,
        )
        
        # Post-hooks
        for hook in self._post_selection_hooks:
            try:
                hook(result)
            except Exception as e:
                logger.error(f"Error in post-selection hook: {e}")
        
        # Registrar
        self._selection_history.append(result)
        if len(self._selection_history) > 1000:
            self._selection_history = self._selection_history[-500:]
        
        self._total_selections += 1
        
        return result
    
    def _evaluate_candidates(
        self,
        candidates: List[ActionCandidate],
        context: SelectionContext,
    ) -> List[ActionCandidate]:
        """Evalua todos los candidatos."""
        for candidate in candidates:
            # Evaluar utilidad
            utility = candidate.utility
            for evaluator in self._utility_evaluators:
                try:
                    utility += evaluator(candidate, context)
                except Exception as e:
                    logger.error(f"Error in utility evaluator: {e}")
            
            # Evaluar riesgo
            risk = candidate.risk
            for evaluator in self._risk_evaluators:
                try:
                    risk += evaluator(candidate, context)
                except Exception as e:
                    logger.error(f"Error in risk evaluator: {e}")
            
            candidate.utility = utility
            candidate.risk = risk
            
            # Calcular score
            candidate.score = self._calculate_score(candidate, context)
        
        return candidates
    
    def _calculate_score(
        self,
        candidate: ActionCandidate,
        context: SelectionContext,
    ) -> float:
        """Calcula puntuacion final."""
        # Componentes base
        utility_component = candidate.utility * self.utility_weight
        risk_component = candidate.risk * self.risk_weight * context.safety_priority
        confidence_component = candidate.confidence * self.confidence_weight
        
        # Score base
        score = utility_component - risk_component + confidence_component
        
        # Modificadores por contexto
        if context.is_safety_mode():
            # En modo seguro, penalizar mas el riesgo
            score -= candidate.risk * 2.0
        
        # Bonus por prioridad
        priority_bonus = (candidate.priority - 5) * 0.1
        score += priority_bonus
        
        # Penalizacion por deadline cercano
        if candidate.deadline_ns:
            remaining_ns = candidate.deadline_ns - time.time_ns()
            if remaining_ns < 0:
                score -= 1.0  # Ya paso el deadline
            elif remaining_ns < 1e9:  # Menos de 1 segundo
                score += 0.5  # Urgente
        
        return score
    
    def _apply_strategy(
        self,
        candidates: List[ActionCandidate],
        context: SelectionContext,
    ) -> Tuple[ActionCandidate, str]:
        """Aplica estrategia de seleccion."""
        import random
        
        if self.strategy == SelectionStrategy.GREEDY:
            best = max(candidates, key=lambda c: c.score)
            return best, f"Seleccion greedy: score={best.score:.3f}"
        
        elif self.strategy == SelectionStrategy.EPSILON_GREEDY:
            if random.random() < context.explore_rate:
                selected = random.choice(candidates)
                self._exploration_count += 1
                return selected, f"Exploracion (epsilon={context.explore_rate})"
            else:
                best = max(candidates, key=lambda c: c.score)
                return best, f"Explotacion greedy: score={best.score:.3f}"
        
        elif self.strategy == SelectionStrategy.SOFTMAX:
            import math
            
            # Calcular probabilidades softmax
            temperature = 0.5
            exp_scores = [math.exp(c.score / temperature) for c in candidates]
            total = sum(exp_scores)
            probs = [e / total for e in exp_scores]
            
            # Samplear
            r = random.random()
            cumsum = 0.0
            for i, prob in enumerate(probs):
                cumsum += prob
                if r < cumsum:
                    return candidates[i], f"Softmax sampling: prob={probs[i]:.3f}"
            
            return candidates[-1], "Softmax fallback"
        
        elif self.strategy == SelectionStrategy.THOMPSON:
            # Thompson sampling simplificado
            import random
            
            sampled_scores = []
            for c in candidates:
                # Samplear de distribucion beta basada en score
                alpha = max(1, c.score * 10 + 1)
                beta = max(1, (1 - c.score) * 10 + 1)
                sample = random.betavariate(alpha, beta)
                sampled_scores.append((c, sample))
            
            best = max(sampled_scores, key=lambda x: x[1])
            return best[0], f"Thompson sampling: sample={best[1]:.3f}"
        
        # Fallback
        return candidates[0], "Fallback selection"
    
    def add_utility_evaluator(
        self,
        evaluator: Callable[[ActionCandidate, SelectionContext], float],
    ) -> None:
        """Agrega evaluador de utilidad."""
        self._utility_evaluators.append(evaluator)
    
    def add_risk_evaluator(
        self,
        evaluator: Callable[[ActionCandidate, SelectionContext], float],
    ) -> None:
        """Agrega evaluador de riesgo."""
        self._risk_evaluators.append(evaluator)
    
    def on_pre_selection(
        self,
        hook: Callable[[List[ActionCandidate]], None],
    ) -> None:
        """Registra hook pre-seleccion."""
        self._pre_selection_hooks.append(hook)
    
    def on_post_selection(
        self,
        hook: Callable[[SelectionResult], None],
    ) -> None:
        """Registra hook post-seleccion."""
        self._post_selection_hooks.append(hook)
    
    def set_strategy(self, strategy: SelectionStrategy) -> None:
        """Cambia estrategia de seleccion."""
        self.strategy = strategy
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        if not self._selection_history:
            return {
                "total_selections": 0,
                "exploration_rate": 0.0,
                "avg_candidates": 0,
                "avg_selection_time_ms": 0,
            }
        
        return {
            "total_selections": self._total_selections,
            "exploration_count": self._exploration_count,
            "exploration_rate": self._exploration_count / max(1, self._total_selections),
            "avg_candidates": sum(len(r.all_candidates) for r in self._selection_history[-100:]) / min(100, len(self._selection_history)),
            "avg_selection_time_ms": sum(r.selection_time_ms for r in self._selection_history[-100:]) / min(100, len(self._selection_history)),
            "strategy": self.strategy.value,
        }
    
    def get_recent_selections(self, limit: int = 10) -> List[Dict[str, Any]]:
        """Obtiene selecciones recientes."""
        return [
            {
                "action": r.selected_action.id if r.selected_action else None,
                "type": r.selected_action.action_type.value if r.selected_action else None,
                "score": r.selected_action.score if r.selected_action else 0,
                "reasoning": r.reasoning,
                "time_ms": r.selection_time_ms,
            }
            for r in self._selection_history[-limit:]
        ]
