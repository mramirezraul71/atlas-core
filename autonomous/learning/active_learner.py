"""
Fase 3 - Active Learning con Curiosity.
Exploración guiada por incertidumbre; identificación de gaps de conocimiento.
TODO: UncertaintyEnsemble, plan_exploration_episode, execute_and_learn.
"""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class ActiveLearner:
    """Aprendizaje activo: estimar incertidumbre, generar queries curiosas, planear exploración."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self._config = config or {}
        self._uncertainty_model = None  # TODO: UncertaintyEnsemble

    def estimate_uncertainty(self, state: Dict[str, Any]) -> float:
        """Devuelve score de incertidumbre 0-1. TODO: ensemble de modelos."""
        # Stub: dominio desconocido = alta incertidumbre
        return 0.5

    def generate_curious_queries(self) -> List[str]:
        """Genera queries/experimentos para reducir gaps. TODO: priorizar por incertidumbre."""
        return []

    def plan_exploration_episode(self, duration_minutes: int = 10) -> Dict[str, Any]:
        """Plan de exploración. TODO: balance exploitation/exploration."""
        return {"actions": [], "duration_minutes": duration_minutes, "stub": True}

    def execute_and_learn(self, exploration_plan: Dict[str, Any]) -> Dict[str, Any]:
        """Ejecuta plan y devuelve reporte de aprendizaje. TODO: ejecución real."""
        return {"learned": [], "stub": True}

    def get_knowledge_gaps(self) -> List[Dict[str, Any]]:
        """Áreas con poca data. TODO: analizar experiencias en semantic memory."""
        return []
