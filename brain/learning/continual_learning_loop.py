"""Loop principal de aprendizaje continuo del robot (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import asyncio
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional


class ContinualLearningLoop:
    """
    Loop: Percibir → Decidir (con detección de incertidumbre) → Actuar → Aprender → Consolidar.
    Soporta tutor IA, lección del día, reporte fin de día y consolidación en background.
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
        ai_tutor: Optional[Any] = None,
    ) -> None:
        self.kb = knowledge_base
        self.uncertainty = uncertainty_detector
        self.ai_consultant = ai_consultant
        self.ai = ai_consultant
        self.semantic_mem = semantic_memory
        self.episodic_mem = episodic_memory
        self.consolidator = consolidator
        self.action_executor = action_executor
        self.ai_tutor = ai_tutor
        self.current_task: Any = None
        self.current_lesson: Optional[Dict[str, Any]] = None
        self.lesson_start_time: Optional[datetime] = None
        self.experience_counter = 0
        self.daily_report: Dict[str, Any] = {
            "tasks_completed": [],
            "challenges": [],
            "knowledge_learned": [],
            "uncertainty_episodes": 0,
            "times_asked_for_help": 0,
        }
        self.background_tasks: List[asyncio.Task[Any]] = []

    def set_action_executor(
        self, executor: Optional[Callable[[str, Dict[str, Any]], Any]]
    ) -> None:
        """Inyectar ejecutor de acciones (p. ej. Cursor run o robot real)."""
        self.action_executor = executor

    async def start_daily_routine(self) -> None:
        """Iniciar rutina diaria: lección del tutor y tareas en background."""
        print("=" * 60)
        print("ATLAS - Iniciando rutina diaria de aprendizaje")
        print("=" * 60)
        if self.ai_tutor and getattr(self.ai_tutor, "assign_daily_lesson", None):
            lesson = self.ai_tutor.assign_daily_lesson()
            if lesson:
                self.current_lesson = lesson
                self.lesson_start_time = datetime.now()
                self.daily_report = {
                    "lesson_id": lesson.get("lesson_id"),
                    "lesson_name": lesson.get("name"),
                    "start_time": self.lesson_start_time.isoformat(),
                    "tasks_completed": [],
                    "challenges": [],
                    "knowledge_learned": [],
                    "uncertainty_episodes": 0,
                    "times_asked_for_help": 0,
                }
                print(f"\nLeccion del dia: {lesson.get('name')}")
                for obj in lesson.get("learning_objectives", []):
                    print(f"   - {obj}")
            else:
                print("\nCurriculum completado. Robot listo para tareas avanzadas.")
                self.current_lesson = None
        else:
            print("\nNo hay tutor IA configurado - modo autonomo")
            self.current_lesson = None
        await self._start_background_tasks()
        print("Rutina diaria iniciada\n")

    async def process_situation(self, situation: Dict[str, Any]) -> Dict[str, Any]:
        """
        Procesar situación con aprendizaje.
        Returns: action_taken, result, learned, asked_for_help, new_knowledge, uncertainty_score
        """
        self.experience_counter += 1
        print(f"\n[EXPERIENCE #{self.experience_counter}] {situation.get('description', 'N/A')[:60]}...")
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
        print(f"  -> Experiencias similares: {len(similar_exp)}")
        action_plan, uncertainty_info = await self._decide_action(
            understanding, similar_exp, situation
        )
        print(f"  -> Incertidumbre: {uncertainty_info.get('score', 0):.2f} - {uncertainty_info.get('reason', '')}")
        asked_for_help = False
        if uncertainty_info.get("is_uncertain"):
            should_ask = getattr(self.uncertainty, "should_ask_for_help", None)
            if should_ask and callable(should_ask):
                if should_ask(
                    uncertainty_info["score"],
                    task_importance=situation.get("risk_level", "normal"),
                ):
                    print("  -> Consultando IA...")
                    guidance = await self._ask_for_help(
                        situation, uncertainty_info, similar_exp[:3]
                    )
                    action_plan = guidance.get("suggested_action") or action_plan
                    for k in guidance.get("new_knowledge") or []:
                        await self._store_new_knowledge([k])
                        self.daily_report.setdefault("knowledge_learned", []).append(k)
                    asked_for_help = True
                    self.daily_report["times_asked_for_help"] = (
                        self.daily_report.get("times_asked_for_help", 0) + 1
                    )
            else:
                guidance = await self._ask_for_help(
                    situation, uncertainty_info, similar_exp
                )
                action_plan = guidance.get("suggested_action") or action_plan
                await self._store_new_knowledge(guidance.get("new_knowledge") or [])
        print(f"  -> Ejecutando: {(action_plan or '')[:50]}...")
        result = await self._execute_action(action_plan, situation)
        print(f"  -> Resultado: {'OK' if result.get('success') else 'FAIL'}")
        learned_knowledge = await self._learn_from_experience(
            situation, action_plan, result, uncertainty_info, asked_for_help
        )
        if self.current_lesson:
            self.daily_report.setdefault("tasks_completed", []).append({
                "task": situation.get("type"),
                "success": result.get("success"),
                "timestamp": datetime.now().isoformat(),
            })
            if uncertainty_info.get("is_uncertain"):
                self.daily_report["uncertainty_episodes"] = (
                    self.daily_report.get("uncertainty_episodes", 0) + 1
                )
        if getattr(self.consolidator, "should_consolidate", lambda f=False: False)(False):
            print("  -> Consolidando conocimiento...")
            await self._consolidate_async()
        return {
            "action_taken": action_plan,
            "result": result,
            "learned": len(learned_knowledge) > 0,
            "asked_for_help": asked_for_help,
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
        confidence = min(len(similar_experiences) / 5.0, 0.95) if similar_experiences else 0.0
        situation_novelty = 1.0 - confidence if len(similar_experiences) == 0 else 0.3
        ensemble_preds = [0.8, 0.75, 0.82] if len(relevant_skills) > 1 else None
        is_uncertain, reason, score = self.uncertainty.is_uncertain(
            confidence=confidence,
            similar_experiences=len(similar_experiences),
            task_name=understanding.get("type"),
            ensemble_predictions=ensemble_preds,
            situation_novelty=situation_novelty,
            context=situation,
        )
        if relevant_skills and not is_uncertain:
            action = relevant_skills[0]
        elif similar_experiences and not is_uncertain:
            action = similar_experiences[0].get("description", "explore_cautiously")
        else:
            action = "request_guidance"
        uncertainty_info = {"is_uncertain": is_uncertain, "reason": reason, "score": score}
        return action, uncertainty_info

    async def _ask_for_help(
        self,
        situation: Dict[str, Any],
        uncertainty_info: Dict[str, Any],
        previous_attempts: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Consultar LLM cuando incierto (ejecuta en thread si es síncrono)."""
        kwargs = {
            "situation": situation.get("description", ""),
            "context": situation,
            "uncertainty_reason": uncertainty_info.get("reason", ""),
            "previous_attempts": previous_attempts[:3] if previous_attempts else None,
            "consultation_type": situation.get("type", "general"),
        }
        if asyncio.iscoroutinefunction(getattr(self.ai, "ask_for_guidance", None)):
            return await self.ai.ask_for_guidance(**kwargs)
        return await asyncio.to_thread(self.ai.ask_for_guidance, **kwargs)

    async def _execute_action(
        self, action_plan: str, situation: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Ejecutar acción: action_executor o simulación si no hay ejecutor."""
        if (
            self.action_executor
            and action_plan
            and str(action_plan).strip().lower() not in ("ask_for_help", "request_guidance")
        ):
            try:
                result = self.action_executor(action_plan, situation)
                if hasattr(result, "__await__"):
                    return await result
                return result if isinstance(result, dict) else {"success": True, "details": result}
            except Exception as e:
                return {"success": False, "error": str(e), "details": "Executor failed"}
        import random
        success_prob = 0.8
        if "complex" in (situation.get("description") or "").lower():
            success_prob = 0.6
        success = random.random() < success_prob
        return {
            "success": success,
            "details": f"Action '{str(action_plan)[:30]}...' executed",
            "timestamp": datetime.now().isoformat(),
        }

    async def _learn_from_experience(
        self,
        situation: Dict[str, Any],
        action: str,
        result: Dict[str, Any],
        uncertainty_info: Dict[str, Any],
        asked_for_help: bool = False,
    ) -> List[Dict[str, Any]]:
        """Aprender de la experiencia y guardar en memoria."""
        new_knowledge: List[Dict[str, Any]] = []
        desc = "%s -> %s" % (situation.get("description", ""), action)
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
                    situation=situation.get("description", ""),
                    context=situation,
                    action_taken=action,
                    result=result,
                    success=bool(result.get("success")),
                    uncertainty_score=uncertainty_info.get("score", 0.0),
                    asked_for_help=asked_for_help,
                    new_knowledge_count=0,
                    task_type=situation.get("type", "general"),
                )
            except Exception:
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
            if getattr(self.uncertainty, "record_success", None):
                self.uncertainty.record_success(situation.get("type", "unknown"))
            new_knowledge.append({
                "type": "success_pattern",
                "situation_type": situation.get("type"),
                "action": action,
                "confidence": 1.0 - uncertainty_info.get("score", 0),
            })
        else:
            if getattr(self.uncertainty, "record_failure", None):
                self.uncertainty.record_failure(situation.get("type", "unknown"))
            new_knowledge.append({
                "type": "failure_pattern",
                "situation_type": situation.get("type"),
                "action": action,
                "reason": result.get("error", "unknown"),
            })
        return new_knowledge

    async def _store_new_knowledge(self, knowledge_items: List[Dict[str, Any]]) -> None:
        """Almacenar nuevo conocimiento en la base de conocimiento."""
        for item in knowledge_items:
            name = item.get("name")
            if not name and item.get("type") == "concept":
                continue
            if item.get("type") == "concept" and name:
                if getattr(self.kb, "add_learned_concept", None):
                    try:
                        self.kb.add_learned_concept(
                            name=name,
                            definition=item.get("definition", ""),
                            concept_type=item.get("concept_type", "learned"),
                        )
                    except Exception:
                        pass
                else:
                    self.kb.concepts[name] = {
                        "definition": item.get("definition", ""),
                        "source": item.get("source", "learned"),
                        "confidence": item.get("confidence", 0.8),
                        "learned_at": datetime.now().isoformat(),
                    }
        if getattr(self.kb, "save_to_disk", None):
            self.kb.save_to_disk()

    async def end_of_day_report(self) -> Optional[Dict[str, Any]]:
        """Reportar al tutor al final del día y obtener evaluación."""
        if not self.ai_tutor or not self.current_lesson:
            return None
        print("\n" + "=" * 60)
        print("📊 FIN DEL DÍA - Generando reporte para tutor...")
        print("=" * 60)
        tasks = self.daily_report.get("tasks_completed", [])
        success_rate = (
            sum(1 for t in tasks if t.get("success")) / len(tasks) if tasks else 0.0
        )
        time_taken = 0.0
        if self.lesson_start_time:
            time_taken = (datetime.now() - self.lesson_start_time).total_seconds() / 3600
        report = {
            "lesson_id": self.current_lesson.get("lesson_id"),
            "tasks_completed": tasks,
            "success_rate": success_rate,
            "time_taken_hours": time_taken,
            "challenges_encountered": self.daily_report.get("challenges", []),
            "knowledge_learned": self.daily_report.get("knowledge_learned", []),
            "uncertainty_episodes": self.daily_report.get("uncertainty_episodes", 0),
            "times_asked_for_help": self.daily_report.get("times_asked_for_help", 0),
            "self_assessment": self._self_assess(success_rate),
        }
        print(f"Tareas: {len(tasks)}, Éxito: {success_rate:.1%}, Ayuda: {report['times_asked_for_help']}")
        if getattr(self.ai_tutor, "review_robot_performance", None):
            evaluation = self.ai_tutor.review_robot_performance(
                lesson_id=self.current_lesson.get("lesson_id"),
                robot_report=report,
            )
            for correction in evaluation.get("knowledge_corrected", []):
                await self._apply_knowledge_correction(correction)
            return evaluation
        return report

    async def _apply_knowledge_correction(self, correction: Dict[str, Any]) -> None:
        """Aplicar corrección de conocimiento indicada por el tutor."""
        concept = correction.get("concept")
        if concept and hasattr(self.kb, "concepts") and concept in self.kb.concepts:
            self.kb.concepts[concept]["definition"] = correction.get("correction", "")
            self.kb.concepts[concept]["corrected_at"] = datetime.now().isoformat()
            if getattr(self.kb, "save_to_disk", None):
                self.kb.save_to_disk()

    def _self_assess(self, success_rate: float) -> str:
        """Auto-evaluación del robot."""
        if success_rate >= 0.9:
            return "Excelente - Dominé esta lección"
        if success_rate >= 0.75:
            return "Bueno - Entendí la mayoría de conceptos"
        if success_rate >= 0.6:
            return "Aceptable - Necesito más práctica"
        return "Difícil - Requiero ayuda adicional"

    async def _start_background_tasks(self) -> None:
        """Iniciar tareas de fondo (p. ej. consolidación periódica)."""
        async def consolidation_loop() -> None:
            try:
                while True:
                    await asyncio.sleep(14400)
                    if getattr(self.consolidator, "should_consolidate", lambda f=False: False)(False):
                        if hasattr(self.consolidator, "consolidate_knowledge"):
                            self.consolidator.consolidate_knowledge()
            except asyncio.CancelledError:
                return

        try:
            # Avoid spawning duplicates if routine is called twice
            if any(t for t in self.background_tasks if not t.done()):
                return
            task = asyncio.create_task(consolidation_loop())
            self.background_tasks.append(task)
        except Exception:
            pass

    async def shutdown(self) -> None:
        """Cancelar tareas en background (útil para tests y apagado limpio)."""
        tasks = [t for t in self.background_tasks if t and not t.done()]
        for t in tasks:
            try:
                t.cancel()
            except Exception:
                pass
        if tasks:
            try:
                await asyncio.gather(*tasks, return_exceptions=True)
            except Exception:
                pass
        self.background_tasks = []

    async def _consolidate_async(self) -> None:
        """Consolidación asíncrona de conocimiento."""
        if hasattr(self.consolidator, "consolidate_knowledge"):
            self.consolidator.consolidate_knowledge()

    def _find_relevant_skills(self, understanding: Dict[str, Any]) -> List[str]:
        """Buscar skills relevantes en la base de conocimiento."""
        task_type = (understanding.get("type") or "").lower()
        relevant: List[str] = []
        for skill_name, skill_def in getattr(self.kb, "skills", {}).items():
            if isinstance(skill_def, dict) and task_type in (skill_def.get("description") or "").lower():
                relevant.append(skill_name)
        return relevant
