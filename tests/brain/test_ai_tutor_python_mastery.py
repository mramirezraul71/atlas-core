from __future__ import annotations


def test_ai_tutor_fallback_generates_python_curriculum_when_goals_indicate_python():
    from brain.learning.ai_tutor import AITutor

    tutor = AITutor(tutor_type="disabled")
    curriculum = tutor.design_curriculum(
        robot_capabilities=["cli", "repo_tools"],
        learning_goals=["python_mastery", "pytest"],
        time_horizon_days=30,
        difficulty_level="progressive",
    )
    assert isinstance(curriculum, list)
    assert len(curriculum) >= 4
    assert curriculum[0]["lesson_id"].startswith("PY")
    assert any("argparse" in (l.get("name", "").lower()) or "scripts" in (l.get("name", "").lower()) for l in curriculum)
    assert all("tasks" in l and isinstance(l["tasks"], list) for l in curriculum)


def test_ai_tutor_fallback_generates_non_python_basic_curriculum_for_other_goals():
    from brain.learning.ai_tutor import AITutor

    tutor = AITutor(tutor_type="disabled")
    curriculum = tutor.design_curriculum(
        robot_capabilities=["vision", "manipulation"],
        learning_goals=["object_recognition"],
        time_horizon_days=30,
        difficulty_level="progressive",
    )
    assert isinstance(curriculum, list)
    assert len(curriculum) >= 1
    assert curriculum[0]["lesson_id"].startswith("L")

