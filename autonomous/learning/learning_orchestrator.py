"""
LearningOrchestrator - Coordina PatternAnalyzer, PerformanceOptimizer, FeedbackLoop, KnowledgeGraph.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from .pattern_analyzer import PatternAnalyzer
from .performance_optimizer import PerformanceOptimizer
from .feedback_loop import FeedbackLoop
from .knowledge_graph import KnowledgeGraph

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class LearningOrchestrator:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("learning", {})
        self._pattern = PatternAnalyzer(self._config)
        self._optimizer = PerformanceOptimizer(self._config)
        self._feedback = FeedbackLoop(self._config)
        self._kg = KnowledgeGraph(self._config)

    def run_learning_cycle(self) -> dict[str, Any]:
        """Ejecuta un ciclo: patrones, optimización, feedback, KG."""
        insights = self._pattern.generate_insights()
        suggestions = self._optimizer.suggest_optimizations()
        areas = self._feedback.identify_improvement_areas()
        return {"insights": insights, "suggestions": suggestions, "improvement_areas": areas}

    def get_learning_insights(self) -> list[str]:
        """Insights recientes."""
        return self.run_learning_cycle().get("insights", [])
