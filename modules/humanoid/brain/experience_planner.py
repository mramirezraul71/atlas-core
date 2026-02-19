"""
Experience-Guided Planner — Planificacion basada en experiencia pasada.

Antes de ejecutar una accion, consulta:
1. Memorias episodicas similares (hippo)
2. Historial de outcomes (world_model)
3. Predictor de resultados
4. Lifelog de acciones pasadas
5. Knowledge base semantica

Genera un plan enriquecido con evidencia historica.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class PlanStep:
    action: str
    params: Dict[str, Any] = field(default_factory=dict)
    expected_outcome: str = ""
    confidence: float = 0.5
    evidence: List[str] = field(default_factory=list)
    alternatives: List[str] = field(default_factory=list)
    risk_level: str = "low"


@dataclass
class ExperiencePlan:
    goal: str
    steps: List[PlanStep] = field(default_factory=list)
    total_confidence: float = 0.5
    similar_episodes_found: int = 0
    historical_success_rate: float = 0.0
    warnings: List[str] = field(default_factory=list)
    recommendations: List[str] = field(default_factory=list)
    created_ts: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "goal": self.goal,
            "steps": [
                {"action": s.action, "params": s.params,
                 "expected_outcome": s.expected_outcome,
                 "confidence": s.confidence, "evidence": s.evidence,
                 "alternatives": s.alternatives, "risk_level": s.risk_level}
                for s in self.steps
            ],
            "total_confidence": self.total_confidence,
            "similar_episodes": self.similar_episodes_found,
            "historical_success_rate": self.historical_success_rate,
            "warnings": self.warnings,
            "recommendations": self.recommendations,
        }


class ExperiencePlanner:
    """Planificador que consulta experiencia pasada antes de actuar."""

    def plan(self, goal: str, context: Dict[str, Any] = None,
             actions: List[str] = None) -> ExperiencePlan:
        plan = ExperiencePlan(goal=goal)
        context = context or {}

        episodes = self._recall_similar_episodes(goal)
        plan.similar_episodes_found = len(episodes)

        outcome_predictions = {}
        for action in (actions or [goal]):
            pred = self._predict_outcome(action, context)
            outcome_predictions[action] = pred

        if episodes:
            successful = [e for e in episodes if e.get("outcome") == "success"]
            plan.historical_success_rate = len(successful) / max(1, len(episodes))

            if plan.historical_success_rate < 0.3:
                plan.warnings.append(
                    f"Tasa de exito historica baja ({plan.historical_success_rate:.0%}). "
                    "Considerar estrategia alternativa."
                )

            successful_actions = self._extract_successful_patterns(successful)
            if successful_actions:
                plan.recommendations.append(
                    f"Patrones exitosos previos: {', '.join(successful_actions[:3])}"
                )

            failed = [e for e in episodes if e.get("outcome") == "failure"]
            failed_actions = self._extract_failure_patterns(failed)
            if failed_actions:
                plan.warnings.append(
                    f"Acciones que han fallado antes: {', '.join(failed_actions[:3])}"
                )

        for action in (actions or [goal]):
            pred = outcome_predictions.get(action, {})
            step = PlanStep(
                action=action,
                params=context,
                expected_outcome=pred.get("prediction", "unknown"),
                confidence=pred.get("success_probability", 0.5),
                risk_level=self._assess_risk(pred),
            )

            relevant = [e for e in episodes if action.lower() in (e.get("goal", "") + str(e.get("actions", ""))).lower()]
            for ep in relevant[:3]:
                step.evidence.append(
                    f"Episodio {ep.get('id', '?')}: {ep.get('outcome', '?')} "
                    f"(reward: {ep.get('reward', 0)})"
                )

            alt = pred.get("alternatives", {})
            if alt and isinstance(alt, dict):
                for a in alt.get("alternatives", [])[:2]:
                    step.alternatives.append(
                        f"{a['action']} (exito: {a['success_rate']:.0%})"
                    )

            plan.steps.append(step)

        if plan.steps:
            plan.total_confidence = sum(s.confidence for s in plan.steps) / len(plan.steps)

        self._log_plan(plan)
        return plan

    def _recall_similar_episodes(self, goal: str) -> List[Dict]:
        results = []
        try:
            from modules.humanoid.hippo import EpisodicMemory
            hippo = EpisodicMemory()
            episodes = hippo.recall_by_goal(goal, limit=10)
            for ep in episodes:
                results.append(ep if isinstance(ep, dict) else ep.to_dict())
        except Exception:
            pass

        try:
            from modules.humanoid.memory_engine.lifelog import get_lifelog
            ll = get_lifelog()
            entries = ll.search(goal, limit=10)
            for e in entries:
                results.append({
                    "id": e.get("id"), "goal": e.get("action", ""),
                    "outcome": "success" if e.get("success") else "failure",
                    "reward": e.get("reward", 0),
                    "actions": e.get("action", ""),
                })
        except Exception:
            pass

        return results

    def _predict_outcome(self, action: str, context: Dict) -> Dict:
        try:
            from modules.humanoid.world_model.predictor import OutcomePredictor
            predictor = OutcomePredictor()
            return predictor.predict(action, context)
        except Exception:
            return {"prediction": "unknown", "success_probability": 0.5, "confidence": 0.0}

    def _extract_successful_patterns(self, episodes: List[Dict]) -> List[str]:
        actions = []
        for ep in episodes:
            if isinstance(ep.get("actions"), list):
                for a in ep["actions"]:
                    at = a.get("action_type", "") if isinstance(a, dict) else str(a)
                    if at and at not in actions:
                        actions.append(at)
            elif isinstance(ep.get("actions"), str) and ep["actions"]:
                if ep["actions"] not in actions:
                    actions.append(ep["actions"])
        return actions

    def _extract_failure_patterns(self, episodes: List[Dict]) -> List[str]:
        return self._extract_successful_patterns(episodes)

    def _assess_risk(self, prediction: Dict) -> str:
        prob = prediction.get("success_probability", 0.5)
        if prob >= 0.8:
            return "low"
        elif prob >= 0.5:
            return "medium"
        else:
            return "high"

    def _log_plan(self, plan: ExperiencePlan):
        try:
            from modules.humanoid.memory_engine.lifelog import get_lifelog
            ll = get_lifelog()
            ll.log_decision(
                source="experience_planner",
                perception=f"Goal: {plan.goal}",
                action=f"Plan generado con {len(plan.steps)} pasos",
                outcome=f"Confianza: {plan.total_confidence:.2f}, "
                        f"Episodios similares: {plan.similar_episodes_found}",
                success=plan.total_confidence > 0.3,
                reward=plan.total_confidence,
            )
        except Exception:
            pass


_instance: Optional[ExperiencePlanner] = None


def get_experience_planner() -> ExperiencePlanner:
    global _instance
    if _instance is None:
        _instance = ExperiencePlanner()
    return _instance
