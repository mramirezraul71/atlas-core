"""Detecta cuándo el robot NO sabe algo y debe pedir ayuda (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import time
from collections import defaultdict
from typing import Any, Dict, List, Optional, Tuple

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


def _std_dev(values: List[float]) -> float:
    if _HAS_NUMPY:
        return float(np.std(values)) if len(values) > 1 else 0.0
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / len(values)
    return variance ** 0.5


class UncertaintyDetector:
    """
    Detecta cuándo el robot NO sabe algo y debe pedir ayuda.

    Estrategias: confidence bajo, ensemble disagreement, sin experiencias similares,
    fallos repetidos, situación nueva, contexto de alto riesgo.
    """

    def __init__(
        self,
        uncertainty_threshold: float = 0.6,
        min_similar_experiences: int = 3,
        failure_window_seconds: int = 300,
    ) -> None:
        self.uncertainty_threshold = uncertainty_threshold
        self.min_similar_experiences = min_similar_experiences
        self.failure_window_seconds = failure_window_seconds
        self.failure_memory: Dict[str, List[float]] = defaultdict(list)
        self.stats: Dict[str, Any] = {
            "total_evaluations": 0,
            "times_uncertain": 0,
            "times_asked_for_help": 0,
            "uncertainty_reasons": defaultdict(int),
        }

    def is_uncertain(
        self,
        confidence: Optional[float] = None,
        similar_experiences: int = 0,
        task_name: Optional[str] = None,
        ensemble_predictions: Optional[List[float]] = None,
        situation_novelty: Optional[float] = None,
        context: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, str, float]:
        """
        Determina si el robot está incierto sobre qué hacer.
        Returns: (is_uncertain, reason, uncertainty_score)
        """
        self.stats["total_evaluations"] += 1
        uncertainty_score = 0.0
        reasons: List[str] = []

        # Factor 1: Confidence bajo
        if confidence is not None:
            if confidence < self.uncertainty_threshold:
                contribution = (self.uncertainty_threshold - confidence) * 1.5
                uncertainty_score += min(contribution, 0.6)
                reasons.append(f"low_confidence_{confidence:.2f}")
                self.stats["uncertainty_reasons"]["low_confidence"] += 1

        # Factor 2: Falta de experiencias similares
        if similar_experiences == 0:
            uncertainty_score += 0.4
            reasons.append("no_similar_experience")
            self.stats["uncertainty_reasons"]["no_experience"] += 1
        elif similar_experiences < self.min_similar_experiences:
            contribution = 0.2 * (1 - similar_experiences / self.min_similar_experiences)
            uncertainty_score += contribution
            reasons.append(f"few_experiences_{similar_experiences}")
            self.stats["uncertainty_reasons"]["few_experiences"] += 1

        # Factor 3: Ensemble disagreement
        if ensemble_predictions and len(ensemble_predictions) > 1:
            std_dev = _std_dev(ensemble_predictions)
            mean_pred = sum(ensemble_predictions) / len(ensemble_predictions)
            if mean_pred > 0:
                cv = std_dev / mean_pred
                if cv > 0.3:
                    uncertainty_score += min(cv, 0.5)
                    reasons.append(f"ensemble_disagreement_cv_{cv:.2f}")
                    self.stats["uncertainty_reasons"]["ensemble_disagreement"] += 1

        # Factor 4: Fallos repetidos
        if task_name:
            recent_failures = self._count_recent_failures(task_name)
            if recent_failures >= 3:
                contribution = min(recent_failures / 10.0, 0.5)
                uncertainty_score += contribution
                reasons.append(f"repeated_failures_{recent_failures}")
                self.stats["uncertainty_reasons"]["repeated_failures"] += 1
            elif recent_failures >= 2:
                uncertainty_score += 0.2
                reasons.append(f"some_failures_{recent_failures}")

        # Factor 5: Situación completamente nueva
        if situation_novelty is not None and situation_novelty > 0.8:
            uncertainty_score += situation_novelty * 0.4
            reasons.append(f"novel_situation_{situation_novelty:.2f}")
            self.stats["uncertainty_reasons"]["novel_situation"] += 1

        # Factor 6: Contexto de alto riesgo
        if context and context.get("risk_level") in ("high", "critical"):
            uncertainty_score += 0.3
            reasons.append(f"high_risk_{context.get('risk_level')}")
            self.stats["uncertainty_reasons"]["high_risk"] += 1

        uncertainty_score = min(uncertainty_score, 1.0)
        is_uncertain = uncertainty_score > 0.5
        if is_uncertain:
            self.stats["times_uncertain"] += 1
        reason = "; ".join(reasons) if reasons else "certain"
        return is_uncertain, reason, uncertainty_score

    def should_ask_for_help(
        self,
        uncertainty_score: float,
        task_importance: str = "normal",
        retry_count: int = 0,
    ) -> bool:
        """Decidir si pedir ayuda según uncertainty, importancia y reintentos."""
        thresholds = {
            "low": 0.8,
            "normal": 0.6,
            "high": 0.4,
            "critical": 0.2,
        }
        base_threshold = thresholds.get(task_importance, 0.6)
        adjusted_threshold = base_threshold - (retry_count * 0.1)
        adjusted_threshold = max(adjusted_threshold, 0.2)
        should_ask = uncertainty_score > adjusted_threshold
        if should_ask:
            self.stats["times_asked_for_help"] += 1
        return should_ask

    def record_failure(self, task_name: str) -> None:
        """Registrar fallo en tarea."""
        self.failure_memory[task_name].append(time.time())
        self._cleanup_old_failures()

    def record_success(self, task_name: str) -> None:
        """Registrar éxito; opcionalmente reducir peso de fallos recientes."""
        if task_name not in self.failure_memory:
            return
        cutoff = time.time() - self.failure_window_seconds
        self.failure_memory[task_name] = [
            t for t in self.failure_memory[task_name] if t > cutoff
        ]
        if not self.failure_memory[task_name]:
            del self.failure_memory[task_name]

    def _count_recent_failures(self, task_name: str) -> int:
        """Contar fallos recientes en ventana de tiempo."""
        if task_name not in self.failure_memory:
            return 0
        now = time.time()
        cutoff = now - self.failure_window_seconds
        return sum(1 for t in self.failure_memory[task_name] if t > cutoff)

    def _cleanup_old_failures(self) -> None:
        """Limpiar fallos muy antiguos (>1 hora)."""
        now = time.time()
        old_cutoff = now - 3600
        for key in list(self.failure_memory.keys()):
            self.failure_memory[key] = [
                t for t in self.failure_memory[key] if t > old_cutoff
            ]
            if not self.failure_memory[key]:
                del self.failure_memory[key]

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas de detección de incertidumbre."""
        total_eval = self.stats["total_evaluations"]
        reasons = self.stats["uncertainty_reasons"]
        if isinstance(reasons, defaultdict):
            reasons = dict(reasons)
        return {
            "total_evaluations": total_eval,
            "times_uncertain": self.stats["times_uncertain"],
            "uncertainty_rate": self.stats["times_uncertain"] / total_eval if total_eval > 0 else 0,
            "times_asked_for_help": self.stats["times_asked_for_help"],
            "help_rate": self.stats["times_asked_for_help"] / total_eval if total_eval > 0 else 0,
            "top_uncertainty_reasons": dict(
                sorted(reasons.items(), key=lambda x: x[1], reverse=True)[:5]
            ),
            "tasks_with_failures": len(self.failure_memory),
            "total_recent_failures": sum(len(f) for f in self.failure_memory.values()),
        }

    def reset_statistics(self) -> None:
        """Reiniciar estadísticas (útil para benchmarking)."""
        self.stats = {
            "total_evaluations": 0,
            "times_uncertain": 0,
            "times_asked_for_help": 0,
            "uncertainty_reasons": defaultdict(int),
        }
