"""Tests para KnowledgeConsolidator y ContinualLearningLoop (PARTE 3: validación)."""
from __future__ import annotations

import tempfile
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ----- KnowledgeConsolidator -----


@pytest.fixture
def mock_semantic_memory():
    m = MagicMock()
    m.recall_similar.return_value = []
    m.add_experience = MagicMock()
    return m


@pytest.fixture
def mock_episodic_memory():
    m = MagicMock()
    m.get_recent_episodes.return_value = []
    m.get_recent = lambda limit=100: []
    m.add_episode = MagicMock(return_value=1)
    return m


@pytest.fixture
def real_kb_temp():
    """KB real en directorio temporal para no tocar initial_kb.json del repo."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "test_kb.json"
        yield str(path)


def test_knowledge_consolidator_should_consolidate_force():
    from brain.learning.knowledge_consolidator import KnowledgeConsolidator
    sem = MagicMock()
    ep = MagicMock()
    ep.get_recent_episodes.return_value = []
    ep.get_recent.return_value = []
    kb = MagicMock()
    kb.concepts = {}
    kb.relations = {}
    kb.rules = {}
    kb.save_to_disk = MagicMock()
    c = KnowledgeConsolidator(semantic_memory=sem, episodic_memory=ep, knowledge_base=kb)
    assert c.should_consolidate(force=True) is True


def test_knowledge_consolidator_get_statistics():
    from brain.learning.knowledge_consolidator import KnowledgeConsolidator
    sem = MagicMock()
    ep = MagicMock()
    ep.get_recent_episodes.return_value = []
    kb = MagicMock()
    kb.concepts = {}
    kb.relations = {}
    kb.rules = {}
    c = KnowledgeConsolidator(semantic_memory=sem, episodic_memory=ep, knowledge_base=kb)
    stats = c.get_statistics()
    assert "total_consolidations" in stats
    assert "patterns_found_total" in stats
    assert "last_consolidation" in stats
    assert "hours_since_last" in stats


def test_knowledge_consolidator_consolidate_returns_report(real_kb_temp):
    from brain.knowledge.initial_knowledge import InitialKnowledgeBase
    from brain.learning.knowledge_consolidator import KnowledgeConsolidator
    kb = InitialKnowledgeBase(storage_path=real_kb_temp)
    sem = MagicMock()
    sem.recall_similar.return_value = []
    ep = MagicMock()
    ep.get_recent_episodes.return_value = []
    ep.get_recent.return_value = []
    c = KnowledgeConsolidator(semantic_memory=sem, episodic_memory=ep, knowledge_base=kb)
    report = c.consolidate_knowledge()
    assert "consolidation_id" in report
    assert "timestamp" in report
    assert "new_concepts" in report
    assert "strengthened_patterns" in report
    assert "generalizations" in report
    assert "updated_relations" in report
    assert "contradictions_resolved" in report


# ----- ContinualLearningLoop -----


@pytest.fixture
def learning_loop_components(mock_semantic_memory, mock_episodic_memory, real_kb_temp):
    from brain.knowledge.initial_knowledge import InitialKnowledgeBase
    from brain.learning.uncertainty_detector import UncertaintyDetector
    from brain.learning.ai_consultant import AIConsultant
    from brain.learning.knowledge_consolidator import KnowledgeConsolidator
    from brain.learning.continual_learning_loop import ContinualLearningLoop
    kb = InitialKnowledgeBase(storage_path=real_kb_temp)
    ud = UncertaintyDetector(uncertainty_threshold=0.6)
    ai = MagicMock()
    ai.ask_for_guidance.return_value = {
        "suggested_action": "move_forward",
        "new_knowledge": [],
        "consultation_successful": True,
    }
    sem = mock_semantic_memory
    ep = mock_episodic_memory
    consolidator = KnowledgeConsolidator(
        semantic_memory=sem, episodic_memory=ep, knowledge_base=kb
    )
    loop = ContinualLearningLoop(
        knowledge_base=kb,
        uncertainty_detector=ud,
        ai_consultant=ai,
        semantic_memory=sem,
        episodic_memory=ep,
        consolidator=consolidator,
    )
    return loop


@pytest.mark.asyncio
async def test_continual_learning_loop_process_situation_returns_structure(learning_loop_components):
    loop = learning_loop_components
    situation = {
        "description": "Objeto en mesa - agarrar",
        "type": "grab",
        "goal": "agarrar objeto",
    }
    result = await loop.process_situation(situation)
    assert "action_taken" in result
    assert "result" in result
    assert "learned" in result
    assert "asked_for_help" in result
    assert "new_knowledge" in result
    assert "uncertainty_score" in result
    assert isinstance(result["result"], dict)
    assert "success" in result["result"]


def test_continual_learning_loop_self_assess():
    from brain.learning.continual_learning_loop import ContinualLearningLoop
    loop = ContinualLearningLoop(
        knowledge_base=MagicMock(),
        uncertainty_detector=MagicMock(),
        ai_consultant=MagicMock(),
        semantic_memory=MagicMock(),
        episodic_memory=MagicMock(),
        consolidator=MagicMock(),
    )
    assert "Excelente" in loop._self_assess(0.92)
    assert "Bueno" in loop._self_assess(0.78)
    assert "Aceptable" in loop._self_assess(0.65)
    assert "Difícil" in loop._self_assess(0.4)


def test_continual_learning_loop_find_relevant_skills():
    from brain.learning.continual_learning_loop import ContinualLearningLoop
    kb = MagicMock()
    kb.skills = {
        "grab_object": {"description": "grab object with gripper"},
        "place_object": {"description": "soltar objeto en posición"},
    }
    loop = ContinualLearningLoop(
        knowledge_base=kb,
        uncertainty_detector=MagicMock(),
        ai_consultant=MagicMock(),
        semantic_memory=MagicMock(),
        episodic_memory=MagicMock(),
        consolidator=MagicMock(),
    )
    relevant = loop._find_relevant_skills({"type": "grab"})
    assert "grab_object" in relevant


# ----- AITutor + ContinualLearningLoop con tutor mock -----


@pytest.fixture
def mock_ai_tutor():
    """Tutor que asigna una leccion y devuelve evaluacion basica."""
    from brain.learning.ai_tutor import AITutor
    tutor = AITutor(tutor_type="disabled")
    tutor.curriculum = [
        {
            "lesson_id": "L001",
            "name": "Leccion de prueba",
            "difficulty": "beginner",
            "duration_hours": 1,
            "prerequisites": [],
            "learning_objectives": ["Objetivo 1"],
            "tasks": [],
            "evaluation_criteria": {},
        },
    ]
    return tutor


@pytest.fixture
def learning_loop_with_tutor(mock_semantic_memory, mock_episodic_memory, real_kb_temp, mock_ai_tutor):
    """Loop con AITutor inyectado (mock con curriculum de fallback)."""
    from brain.knowledge.initial_knowledge import InitialKnowledgeBase
    from brain.learning.uncertainty_detector import UncertaintyDetector
    from brain.learning.ai_consultant import AIConsultant
    from brain.learning.knowledge_consolidator import KnowledgeConsolidator
    from brain.learning.continual_learning_loop import ContinualLearningLoop
    kb = InitialKnowledgeBase(storage_path=real_kb_temp)
    ud = UncertaintyDetector(uncertainty_threshold=0.6)
    ai = MagicMock()
    ai.ask_for_guidance.return_value = {"suggested_action": "move_forward", "new_knowledge": []}
    sem = mock_semantic_memory
    ep = mock_episodic_memory
    consolidator = KnowledgeConsolidator(semantic_memory=sem, episodic_memory=ep, knowledge_base=kb)
    loop = ContinualLearningLoop(
        knowledge_base=kb,
        uncertainty_detector=ud,
        ai_consultant=ai,
        semantic_memory=sem,
        episodic_memory=ep,
        consolidator=consolidator,
        ai_tutor=mock_ai_tutor,
    )
    return loop


@pytest.mark.asyncio
async def test_continual_learning_loop_with_tutor_start_daily_routine(learning_loop_with_tutor, mock_ai_tutor):
    """Con tutor mock, start_daily_routine asigna current_lesson y daily_report."""
    loop = learning_loop_with_tutor
    try:
        await loop.start_daily_routine()
        assert loop.current_lesson is not None
        assert loop.current_lesson["lesson_id"] == "L001"
        assert loop.lesson_start_time is not None
        assert "lesson_id" in loop.daily_report
        assert loop.daily_report["lesson_id"] == "L001"
    finally:
        await loop.shutdown()


@pytest.mark.asyncio
async def test_continual_learning_loop_with_tutor_end_of_day_report(learning_loop_with_tutor, mock_ai_tutor):
    """Con tutor y leccion activa, end_of_day_report devuelve evaluacion (fallback)."""
    loop = learning_loop_with_tutor
    try:
        await loop.start_daily_routine()
        loop.daily_report["tasks_completed"] = [
            {"task": "grab", "success": True, "timestamp": "2025-01-01T12:00:00"},
        ]
        evaluation = await loop.end_of_day_report()
        assert evaluation is not None
        assert "score" in evaluation
        assert "passed" in evaluation
        assert "feedback" in evaluation
    finally:
        await loop.shutdown()
