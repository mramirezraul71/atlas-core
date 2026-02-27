from __future__ import annotations

from datetime import datetime
from unittest.mock import MagicMock

import pytest


@pytest.mark.asyncio
async def test_end_of_day_report_uses_python_mastery_evaluator_and_records_offline_evaluation(monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.continual_learning_loop import ContinualLearningLoop

    tutor = AITutor(tutor_type="disabled")
    # Simula currículum con una lección PY activa
    tutor.curriculum = [{"lesson_id": "PY001", "name": "Python Core", "learning_objectives": [], "tasks": []}]

    # Patch del evaluador para evitar correr pytest real en unit tests
    def _fake_eval(*, lesson_id: str, repo_root, timeout_s: int = 180):
        return {"lesson_id": lesson_id, "score": 80, "passed": True, "knowledge_corrected": [], "evidence": {"fake": True}}

    monkeypatch.setattr("brain.learning.python_mastery_evaluator.evaluate_python_mastery_lesson", _fake_eval)

    loop = ContinualLearningLoop(
        knowledge_base=MagicMock(),
        uncertainty_detector=MagicMock(),
        ai_consultant=MagicMock(),
        semantic_memory=MagicMock(),
        episodic_memory=MagicMock(),
        consolidator=MagicMock(),
        action_executor=None,
        ai_tutor=tutor,
    )
    loop.current_lesson = {"lesson_id": "PY001", "name": "Python Core"}
    loop.lesson_start_time = datetime.now()
    loop.daily_report["tasks_completed"] = [{"task": "py001", "success": True, "timestamp": "2026-01-01T00:00:00"}]

    evaluation = await loop.end_of_day_report()
    assert evaluation is not None
    assert evaluation.get("score") == 80
    assert evaluation.get("passed") is True
    assert any(l.get("lesson_id") == "PY001" for l in tutor.completed_lessons)

