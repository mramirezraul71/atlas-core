"""Loop principal de aprendizaje continuo del robot (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

from datetime import datetime
from typing import Any, Callable, Dict, List, Optional


class ContinualLearningLoop:
    """
    Loop: Percibir → Decidir (con detección de incertidumbre) → Actuar → Aprender → Consolidar.
    """

    def __init__(
        self,
        knowledge_base: Any,
        uncertainty_detector: Any,
        ai_consultant: Any,
        semantic_memory: Any,
        episodic_memory: Any,
        consolidator: Any,
        action_executor: Optional[Callable[[str, Dict[str, Any]], Any]] = None,
    ) -> None:
        self.kb = knowledge_base
        self.uncertainty = uncertainty_detector
        self.ai = ai_consultant
        self.semantic_mem = semantic_memory
        self.episodic_mem = episodic_memory
        self.consolidator = consolidator
        self.action_executor = action_executor
        self.current_task: Any = None
        self.experience_counter = 0

    def set_action_executor(
        self, executor: Optional[Callable[[str, Dict[str, Any]], Any]]
    ) -> None:
        """Inyectar ejecutor de acciones (p. ej. Cursor run o robot real)."""
        self.action_executor = executor

    async def process_situation(self, situation: Dict[str, Any]) -> Dict[str, Any]:
        """
        Procesar situación con aprendizaje.
        Returns: action_taken, result, learned, asked_for_help, new_knowledge, uncertainty_score
        """
        understanding = await self._understand_situation(situation)
        similar_exp: List[Dict[str, Any]] = []
        if getattr(self.semantic_mem, "recall_similar", None):
            try:
                similar_exp = self.semantic_mem.recall_similar(
                    situation.get("description", ""),
                    top_k=5,
                    min_similarity=0.7,
                )
            except Exception:
                pass
        action_plan, uncertainty_info = await self._decide_action(
            understanding, similar_exp, situation
        )
        if uncertainty_info.get("is_uncertain"):
            guidance = await self._ask_for_help(
                situation, uncertainty_info, similar_exp
            )
            action_plan = guidance.get("suggested_action") or action_plan
            await self._store_new_knowledge(guidance.get("new_knowledge") or [])
        result = await self._execute_action(action_plan, situation)
        learned_knowledge = await self._learn_from_experience(
            situation, action_plan, result, uncertainty_info
        )
        if self.consolidator.should_consolidate():
            self.consolidator.consolidate_knowledge()
        return {
            "action_taken": action_plan,
            "result": result,
            "learned": len(learned_knowledge) > 0,
            "asked_for_help": uncertainty_info.get("is_uncertain", False),
            "new_knowledge": learned_knowledge,
            "uncertainty_score": uncertainty_info.get("score", 0.0),
        }

    async def _understand_situation(self, situation: Dict[str, Any]) -> Dict[str, Any]:
        """Entender y clasificar situación."""
        return {
            "type": situation.get("type", "unknown"),
            "entities": situation.get("entities", []),
            "goal": situation.get("goal"),
            "constraints": situation.get("constraints", []),
        }

    async def _decide_action(
        self,
        understanding: Dict[str, Any],
        similar_experiences: List[Dict[str, Any]],
        situation: Dict[str, Any],
    ) -> tuple:
        """Decidir acción con detección de incertidumbre. Returns (action_plan, uncertainty_info)."""
        relevant_skills = self._find_relevant_skills(understanding)
        confidence = min(1.0, len(similar_experiences) / 5.0) if similar_experiences else 0.0
        ensemble_preds = [0.8, 0.75, 0.82] if len(relevant_skills) > 1 else None
        is_uncertain, reason, score = self.uncertainty.is_uncertain(
            confidence=confidence,
            similar_experiences=len(similar_experiences),
            task_name=understanding.get("type"),
            ensemble_predictions=ensemble_preds,
        )
        action = relevant_skills[0] if relevant_skills and not is_uncertain else "ask_for_help"
        uncertainty_info = {"is_uncertain": is_uncertain, "reason": reason, "score": score}
        return action, uncertainty_info

    async def _ask_for_help(
        self,
        situation: Dict[str, Any],
        uncertainty_info: Dict[str, Any],
        previous_attempts: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Consultar LLM cuando incierto."""
        return self.ai.ask_for_guidance(
            situation=situation.get("description", ""),
            context=situation,
            uncertainty_reason=uncertainty_info.get("reason", ""),
            previous_attempts=previous_attempts,
        )

    async def _execute_action(
        self, action_plan: str, situation: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Ejecutar acción: usa action_executor si está definido y no es ask_for_help."""
        if (
            self.action_executor
            and action_plan
            and str(action_plan).strip().lower() != "ask_for_help"
        ):
            try:
                result = self.action_executor(action_plan, situation)
                if hasattr(result, "__await__"):
                    return await result
                return result if isinstance(result, dict) else {"success": True, "details": result}
            except Exception as e:
                return {"success": False, "error": str(e), "details": "Executor failed"}
        return {"success": True, "details": "Executed: %s" % (action_plan or "none")}

    async def _learn_from_experience(
        self,
        situation: Dict[str, Any],
        action: str,
        result: Dict[str, Any],
        uncertainty_info: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        """Aprender de la experiencia y guardar en memoria."""
        new_knowledge: List[Dict[str, Any]] = []
        desc = "%s → %s" % (situation.get("description", ""), action)
        if getattr(self.semantic_mem, "add_experience", None):
            try:
                self.semantic_mem.add_experience(
                    description=desc,
                    context=str(situation)[:2000],
                    outcome=str(result),
                    tags=[situation.get("type", "general"), "learned"],
                )
            except Exception:
                pass
        if getattr(self.episodic_mem, "add_episode", None):
            try:
                self.episodic_mem.add_episode(
                    situation_type=situation.get("type", "general"),
                    description=situation.get("description", ""),
                    action=action,
                    outcome=str(result),
                    success=bool(result.get("success")),
                    context=situation,
                )
            except Exception:
                pass
        if result.get("success"):
            new_knowledge.append({
                "type": "success_pattern",
                "situation_type": situation.get("type"),
                "action": action,
                "confidence": 1.0 - uncertainty_info.get("score", 0),
            })
        else:
            self.uncertainty.record_failure(situation.get("type", "unknown"))
            new_knowledge.append({
                "type": "failure_pattern",
                "situation_type": situation.get("type"),
                "action": action,
                "reason": result.get("error", "unknown"),
            })
        self.experience_counter += 1
        return new_knowledge

    async def _store_new_knowledge(self, knowledge_items: List[Dict[str, Any]]) -> None:
        """Almacenar nuevo conocimiento en la base de conocimiento."""
        for item in knowledge_items:
            if item.get("type") == "concept" and "name" in item:
                name = item["name"]
                self.kb.concepts[name] = {
                    "definition": item.get("definition", ""),
                    "source": item.get("source", "learned"),
                    "confidence": item.get("confidence", 0.8),
                    "learned_at": datetime.now().isoformat(),
                }
        self.kb.save_to_disk()

    def _find_relevant_skills(self, understanding: Dict[str, Any]) -> List[str]:
        """Buscar skills relevantes en la base de conocimiento."""
        task_type = (understanding.get("type") or "").lower()
        relevant: List[str] = []
        for skill_name, skill_def in getattr(self.kb, "skills", {}).items():
            if isinstance(skill_def, dict) and task_type in (skill_def.get("description") or "").lower():
                relevant.append(skill_name)
        return relevant
