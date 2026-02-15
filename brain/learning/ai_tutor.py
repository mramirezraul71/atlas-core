"""IA Tutor (Claude Opus / GPT-4) que guía el aprendizaje del robot ATLAS."""
from __future__ import annotations

import json
import os
import re
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

try:
    import requests
except ImportError:
    requests = None


class AITutor:
    """
    IA Tutor que guía el aprendizaje del robot.

    Funciones: diseñar curriculum, asignar lecciones progresivas, evaluar desempeño,
    validar conocimiento, adaptar dificultad, sugerir exploración, identificar fortalezas/debilidades.
    """

    def __init__(
        self,
        tutor_type: str = "claude",
        api_key: Optional[str] = None,
        review_interval_hours: int = 6,
        use_local_fallback: bool = True,
    ):
        self.tutor_type = tutor_type
        self.api_key = api_key or os.getenv("ANTHROPIC_API_KEY") or os.getenv("OPENAI_API_KEY")
        self.review_interval = timedelta(hours=review_interval_hours)
        self.use_local_fallback = use_local_fallback

        self.curriculum: List[Dict[str, Any]] = []
        self.current_lesson: Optional[Dict[str, Any]] = None
        self.completed_lessons: List[Dict[str, Any]] = []
        self.failed_lessons: List[Dict[str, Any]] = []

        self.robot_profile: Dict[str, Any] = {
            "student_id": "ATLAS_ROBOT_001",
            "knowledge_level": "beginner",
            "learning_speed": "medium",
            "strengths": [],
            "weaknesses": [],
            "total_lessons_completed": 0,
            "total_lessons_failed": 0,
            "average_score": 0.0,
            "preferred_learning_style": "experiential",
            "created_at": datetime.now().isoformat(),
        }

        self.tutor_sessions: List[Dict[str, Any]] = []
        self.last_review: Optional[datetime] = None
        self.last_curriculum_update: Optional[datetime] = None

        self.stats: Dict[str, Any] = {
            "total_evaluations": 0,
            "total_feedback_given": 0,
            "total_corrections_made": 0,
            "total_api_calls": 0,
            "total_api_cost_usd": 0.0,
        }

        if tutor_type == "claude":
            self.api_endpoint = "https://api.anthropic.com/v1/messages"
            self.model = "claude-sonnet-4-20250514"
        elif tutor_type == "gpt4":
            self.api_endpoint = "https://api.openai.com/v1/chat/completions"
            self.model = "gpt-4-turbo"
        else:
            self.api_endpoint = ""
            self.model = ""

    def design_curriculum(
        self,
        robot_capabilities: List[str],
        learning_goals: List[str],
        time_horizon_days: int = 30,
        difficulty_level: str = "progressive",
    ) -> List[Dict[str, Any]]:
        """Diseña curriculum personalizado (llamada a IA o fallback)."""
        print(f"[AI TUTOR] Diseñando curriculum para {time_horizon_days} días...")

        prompt = f"""Eres un tutor experto en robótica. Tu estudiante es el robot ATLAS.

PERFIL: {self.robot_profile['knowledge_level']}, fortalezas: {self.robot_profile['strengths']}, debilidades: {self.robot_profile['weaknesses']}.
CAPACIDADES: {json.dumps(robot_capabilities)}
OBJETIVOS: {json.dumps(learning_goals)}
DIFICULTAD: {difficulty_level}

Genera un curriculum en JSON: lista de lecciones con lesson_id, name, difficulty, duration_hours, prerequisites, description, learning_objectives, tasks (task_id, description, success_criteria), evaluation_criteria, next_lessons.
Genera {min(time_horizon_days // 2, 15)} lecciones. Responde solo con un array JSON válido."""

        response = self._call_tutor_ai(prompt, max_tokens=4096)

        if response and "error" not in response.lower():
            curriculum = self._extract_json_from_response(response)
            if isinstance(curriculum, list) and len(curriculum) > 0:
                self.curriculum = curriculum
                self.last_curriculum_update = datetime.now()
                print(f"[AI TUTOR] OK Curriculum creado: {len(curriculum)} lecciones")
                return curriculum

        print("[AI TUTOR] Warning: Error disenando curriculum, usando fallback")
        self.curriculum = self._generate_basic_curriculum()
        return self.curriculum

    def assign_daily_lesson(self) -> Optional[Dict[str, Any]]:
        """Asigna la lección del día según progreso y prerequisitos."""
        if not self.curriculum:
            print("[AI TUTOR] No hay curriculum. Generando uno básico...")
            self.design_curriculum(
                robot_capabilities=["vision", "manipulation"],
                learning_goals=["object_recognition", "basic_manipulation"],
            )

        completed_ids = [l["lesson_id"] for l in self.completed_lessons]

        for lesson in self.curriculum:
            lesson_id = lesson["lesson_id"]
            if lesson_id in completed_ids:
                continue
            prereqs = lesson.get("prerequisites", [])
            if all(p in completed_ids for p in prereqs):
                self.current_lesson = lesson
                print(f"[AI TUTOR] Leccion asignada: {lesson['name']}")
                for obj in lesson.get("learning_objectives", []):
                    print(f"  - {obj}")
                return lesson

        print("[AI TUTOR] Curriculum completado!")
        return None

    def review_robot_performance(self, lesson_id: str, robot_report: Dict[str, Any]) -> Dict[str, Any]:
        """Evalúa el desempeño del robot y devuelve evaluación con feedback y correcciones."""
        print(f"[AI TUTOR] Evaluando desempeño en lección {lesson_id}...")
        self.stats["total_evaluations"] += 1

        lesson = next((l for l in self.curriculum if l["lesson_id"] == lesson_id), None)
        if not lesson:
            return {"error": f"Lesson {lesson_id} not found", "score": 0, "passed": False}

        prompt = f"""Eres un tutor evaluando al robot ATLAS.

LECCIÓN: {json.dumps(lesson, indent=2)}
REPORTE DEL ROBOT: {json.dumps(robot_report, indent=2)}

Evalúa y responde en JSON con: score (0-100), evaluation (texto), grade, passed (bool), feedback, strengths_demonstrated, areas_for_improvement, knowledge_validated (lista con concept, validation, confidence), knowledge_corrected (lista con concept, wrong_belief, correction, explanation), next_steps, adjust_difficulty, repeat_lesson, learning_speed_assessment. Responde solo JSON válido."""

        response = self._call_tutor_ai(prompt, max_tokens=2048)

        if response and "error" not in response.lower():
            evaluation = self._extract_json_from_response(response)
            if isinstance(evaluation, dict) and "score" in evaluation:
                self._update_robot_profile(evaluation, lesson)
                self.tutor_sessions.append({
                    "session_id": f"session_{len(self.tutor_sessions) + 1}",
                    "timestamp": datetime.now().isoformat(),
                    "lesson_id": lesson_id,
                    "evaluation": evaluation,
                    "robot_report": robot_report,
                })
                self.stats["total_feedback_given"] += 1
                self.stats["total_corrections_made"] += len(evaluation.get("knowledge_corrected", []))

                score = evaluation.get("score", 0)
                if score >= 60:
                    self.completed_lessons.append({
                        "lesson_id": lesson_id,
                        "completed_at": datetime.now().isoformat(),
                        "score": score,
                        "attempts": 1,
                    })
                    print(f"[AI TUTOR] Leccion completada: {score}/100")
                else:
                    self.failed_lessons.append({
                        "lesson_id": lesson_id,
                        "failed_at": datetime.now().isoformat(),
                        "score": score,
                    })
                    print(f"[AI TUTOR] Leccion fallada: {score}/100")
                self.last_review = datetime.now()
                return evaluation

        print("[AI TUTOR] Warning: Error en evaluacion, usando fallback")
        return self._generate_basic_evaluation(robot_report)

    def validate_learned_knowledge(self, knowledge_items: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Valida items de conocimiento aprendidos (correcto/parcial/incorrecto)."""
        if not knowledge_items:
            return []

        print(f"[AI TUTOR] Validando {len(knowledge_items)} items...")
        prompt = f"""Valida cada concepto que el robot ATLAS aprendió. Responde JSON array con original_concept, validation (correct|incorrect|partial), confidence, correction (si incorrect), refined_definition (si partial).

CONOCIMIENTO: {json.dumps(knowledge_items, indent=2)}"""

        response = self._call_tutor_ai(prompt, max_tokens=1536)
        if response:
            validated = self._extract_json_from_response(response)
            if isinstance(validated, list):
                return validated
        return [
            {**item, "validation": "needs_human_review", "tutor_unavailable": True}
            for item in knowledge_items
        ]

    def suggest_exploration_task(
        self,
        robot_strengths: List[str],
        robot_weaknesses: List[str],
        recent_experiences: List[str],
    ) -> Dict[str, Any]:
        """Sugiere tarea de exploración (zona de desarrollo próximo)."""
        prompt = f"""Diseña una tarea de exploración para ATLAS. Fortalezas: {robot_strengths}. Debilidades: {robot_weaknesses}. Experiencias recientes: {recent_experiences[-5:]}. Responde JSON: task_name, description, learning_objective, difficulty, estimated_duration_minutes, safety_precautions, success_criteria."""

        response = self._call_tutor_ai(prompt)
        if response:
            task = self._extract_json_from_response(response)
            if isinstance(task, dict):
                print(f"[AI TUTOR] Tarea sugerida: {task.get('task_name', 'Exploracion')}")
                return task
        return self._generate_basic_exploration_task()

    def _call_tutor_ai(self, prompt: str, max_tokens: int = 2048) -> str:
        if self.tutor_type == "disabled":
            return "Tutor disabled"
        if not self.api_key:
            print("[AI TUTOR] No API key - usando fallback local")
            if self.use_local_fallback:
                return self._call_local_llm(prompt)
            return "No API key"
        if not requests:
            return "requests not installed"

        try:
            self.stats["total_api_calls"] += 1
            if self.tutor_type == "claude":
                headers = {
                    "x-api-key": self.api_key,
                    "anthropic-version": "2023-06-01",
                    "content-type": "application/json",
                }
                data = {
                    "model": self.model,
                    "max_tokens": max_tokens,
                    "messages": [{"role": "user", "content": prompt}],
                }
                resp = requests.post(self.api_endpoint, headers=headers, json=data, timeout=120)
                if resp.status_code == 200:
                    result = resp.json()["content"][0]["text"]
                    inp = len(prompt) / 4
                    out = len(result) / 4
                    self.stats["total_api_cost_usd"] += (inp * 0.000015) + (out * 0.000075)
                    return result
                if self.use_local_fallback:
                    return self._call_local_llm(prompt)
            elif self.tutor_type == "gpt4":
                headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
                data = {"model": self.model, "max_tokens": max_tokens, "messages": [{"role": "user", "content": prompt}]}
                resp = requests.post(self.api_endpoint, headers=headers, json=data, timeout=120)
                if resp.status_code == 200:
                    result = resp.json()["choices"][0]["message"]["content"]
                    return result
                if self.use_local_fallback:
                    return self._call_local_llm(prompt)
        except Exception as e:
            print(f"[AI TUTOR] Exception: {e}")
            if self.use_local_fallback:
                return self._call_local_llm(prompt)
        return "Error calling tutor"

    def _call_local_llm(self, prompt: str) -> str:
        if not requests:
            return ""
        try:
            resp = requests.post(
                "http://localhost:11434/api/generate",
                json={"model": "llama3.2", "prompt": prompt, "stream": False},
                timeout=90,
            )
            if resp.status_code == 200:
                return resp.json().get("response", "")
        except Exception:
            pass
        return "Tutor unavailable"

    def _extract_json_from_response(self, response: str) -> Any:
        if not response:
            return {}
        json_match = re.search(r"```json\s*(.*?)\s*```", response, re.DOTALL)
        if json_match:
            try:
                return json.loads(json_match.group(1).strip())
            except Exception:
                pass
        json_match = re.search(r"(\[.*\]|\{.*\})", response, re.DOTALL)
        if json_match:
            try:
                return json.loads(json_match.group(1))
            except Exception:
                pass
        try:
            return json.loads(response)
        except Exception:
            return {"error": "Could not parse JSON", "raw": response[:500]}

    def _update_robot_profile(self, evaluation: Dict[str, Any], lesson: Dict[str, Any]) -> None:
        score = evaluation.get("score", 0)
        passed = evaluation.get("passed", score >= 60)
        if passed:
            self.robot_profile["total_lessons_completed"] = self.robot_profile.get("total_lessons_completed", 0) + 1
        else:
            self.robot_profile["total_lessons_failed"] = self.robot_profile.get("total_lessons_failed", 0) + 1

        total = self.robot_profile["total_lessons_completed"] + self.robot_profile.get("total_lessons_failed", 0)
        if total > 0:
            current_avg = self.robot_profile["average_score"]
            new_avg = (current_avg * (total - 1) + score) / total
            self.robot_profile["average_score"] = round(new_avg, 1)

        for s in evaluation.get("strengths_demonstrated", []):
            if s not in self.robot_profile["strengths"]:
                self.robot_profile["strengths"].append(s)
        self.robot_profile["weaknesses"] = evaluation.get("areas_for_improvement", self.robot_profile["weaknesses"])

        if self.robot_profile["average_score"] >= 90:
            self.robot_profile["knowledge_level"] = "advanced"
        elif self.robot_profile["average_score"] >= 75:
            self.robot_profile["knowledge_level"] = "intermediate"
        else:
            self.robot_profile["knowledge_level"] = "beginner"

        speed = evaluation.get("learning_speed_assessment")
        if speed:
            self.robot_profile["learning_speed"] = speed

    def _generate_basic_curriculum(self) -> List[Dict[str, Any]]:
        return [
            {
                "lesson_id": "L001",
                "name": "Reconocimiento Visual Básico",
                "difficulty": "beginner",
                "duration_hours": 4,
                "prerequisites": [],
                "learning_objectives": ["Identificar objetos comunes"],
                "tasks": [
                    {"task_id": "T001", "description": "Identificar 20 objetos", "success_criteria": "accuracy >= 0.7"},
                ],
                "evaluation_criteria": {"excellent": "score >= 90", "good": "score >= 70", "acceptable": "score >= 60"},
                "next_lessons": [],
            },
        ]

    def _generate_basic_evaluation(self, report: Dict[str, Any]) -> Dict[str, Any]:
        success_rate = report.get("success_rate", 0.5)
        score = min(100, int(success_rate * 100))
        return {
            "score": score,
            "evaluation": "Bueno" if score >= 70 else "Aceptable",
            "grade": "A" if score >= 90 else "B" if score >= 75 else "C" if score >= 60 else "D",
            "passed": score >= 60,
            "feedback": f"Tasa de éxito: {success_rate:.1%}",
            "next_steps": "Continuar con siguiente lección",
            "adjust_difficulty": "maintain",
            "knowledge_corrected": [],
        }

    def _generate_basic_exploration_task(self) -> Dict[str, Any]:
        return {
            "task_name": "Exploración Libre",
            "description": "Observar entorno y registrar objetos nuevos",
            "learning_objective": "Expandir conocimiento de objetos",
            "estimated_duration_minutes": 30,
            "success_criteria": "Identificar 5+ objetos no vistos antes",
        }

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "tutor_type": self.tutor_type,
            "total_lessons_in_curriculum": len(self.curriculum),
            "lessons_completed": len(self.completed_lessons),
            "lessons_failed": len(self.failed_lessons),
            "completion_rate": len(self.completed_lessons) / len(self.curriculum) if self.curriculum else 0,
            "robot_average_score": self.robot_profile.get("average_score", 0),
            "robot_level": self.robot_profile.get("knowledge_level", "beginner"),
            "total_evaluations": self.stats["total_evaluations"],
            "total_api_calls": self.stats["total_api_calls"],
            "total_api_cost_usd": round(self.stats["total_api_cost_usd"], 4),
            "last_review": self.last_review.isoformat() if self.last_review else None,
        }
