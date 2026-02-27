from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock


def test_campaign_start_picks_first_pending_lesson_and_persists(tmp_path: Path, monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.python_mastery_campaign import campaign_state_path, start_or_resume_campaign

    # Fijar repo_root al tmp para no tocar snapshots reales
    monkeypatch.setattr("brain.learning.python_mastery_campaign.campaign_state_path", lambda repo_root=None: tmp_path / "state.json")

    tutor = AITutor(tutor_type="disabled")
    loop = MagicMock()
    res = start_or_resume_campaign(tutor=tutor, loop=loop, repo_root=tmp_path, reset=True)

    assert res["state"]["status"] in ("active", "completed")
    assert (tmp_path / "state.json").exists()
    if res["current_lesson"]:
        assert res["current_lesson"]["lesson_id"].startswith("PY")


def test_campaign_step_advances_on_pass(tmp_path: Path, monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.python_mastery_campaign import campaign_step, start_or_resume_campaign

    monkeypatch.setattr("brain.learning.python_mastery_campaign.campaign_state_path", lambda repo_root=None: tmp_path / "state.json")

    # Evaluador fake: PY001 pasa, resto no importa
    def fake_eval(*, lesson_id: str, repo_root, timeout_s: int = 180):
        return {"lesson_id": lesson_id, "score": 80, "passed": True, "knowledge_corrected": []}

    monkeypatch.setattr("brain.learning.python_mastery_campaign.evaluate_python_mastery_lesson", fake_eval, raising=False)
    # Patch import interno: el campaign_step importa desde python_mastery_evaluator,
    # así que monkeypatcheamos allí.
    monkeypatch.setattr("brain.learning.python_mastery_evaluator.evaluate_python_mastery_lesson", fake_eval)

    tutor = AITutor(tutor_type="disabled")
    loop = MagicMock()
    start_or_resume_campaign(tutor=tutor, loop=loop, repo_root=tmp_path, reset=True)

    res = campaign_step(tutor=tutor, loop=loop, repo_root=tmp_path, lesson_id="PY001", timeout_s=10)
    assert res["ok"] is True
    assert res["evaluation"]["passed"] is True
    # al pasar, debería intentar asignar siguiente
    assert res["state"]["status"] in ("active", "completed")
    assert any(x.get("lesson_id") == "PY001" for x in tutor.completed_lessons)


def test_campaign_step_retries_until_pass(tmp_path: Path, monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.python_mastery_campaign import campaign_step, start_or_resume_campaign

    monkeypatch.setattr("brain.learning.python_mastery_campaign.campaign_state_path", lambda repo_root=None: tmp_path / "state.json")

    calls = {"n": 0}

    def fake_eval(*, lesson_id: str, repo_root, timeout_s: int = 180):
        calls["n"] += 1
        if calls["n"] < 2:
            return {"lesson_id": lesson_id, "score": 20, "passed": False, "knowledge_corrected": []}
        return {"lesson_id": lesson_id, "score": 80, "passed": True, "knowledge_corrected": []}

    monkeypatch.setattr("brain.learning.python_mastery_evaluator.evaluate_python_mastery_lesson", fake_eval)
    monkeypatch.setattr("brain.learning.python_mastery_campaign.time.sleep", lambda s: None)

    tutor = AITutor(tutor_type="disabled")
    loop = MagicMock()
    start_or_resume_campaign(tutor=tutor, loop=loop, repo_root=tmp_path, reset=True)

    res = campaign_step(tutor=tutor, loop=loop, repo_root=tmp_path, lesson_id="PY001", max_attempts=2, backoff_s=0.0)
    assert res["ok"] is True
    assert res["attempts_run"] == 2
    assert res["evaluation"]["passed"] is True


def test_campaign_step_generates_remediation_plan_on_fail(tmp_path: Path, monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.python_mastery_campaign import campaign_step, start_or_resume_campaign

    monkeypatch.setattr("brain.learning.python_mastery_campaign.campaign_state_path", lambda repo_root=None: tmp_path / "state.json")
    monkeypatch.setattr("brain.learning.python_mastery_campaign.time.sleep", lambda s: None)

    def fake_eval(*, lesson_id: str, repo_root, timeout_s: int = 180):
        return {
            "lesson_id": lesson_id,
            "score": 20,
            "passed": False,
            "evidence": {
                "checks": [
                    {"name": "required_files", "ok": False, "missing": ["training/python_mastery/utils.py"], "required": []},
                    {"name": "pytest", "ok": False, "nodeids": ["tests/python_mastery/test_py001_utils.py"], "result": {"output": "NotImplementedError in training/python_mastery/utils.py"}},
                ]
            },
        }

    monkeypatch.setattr("brain.learning.python_mastery_evaluator.evaluate_python_mastery_lesson", fake_eval)

    tutor = AITutor(tutor_type="disabled")
    loop = MagicMock()
    start_or_resume_campaign(tutor=tutor, loop=loop, repo_root=tmp_path, reset=True)

    res = campaign_step(
        tutor=tutor,
        loop=loop,
        repo_root=tmp_path,
        lesson_id="PY001",
        max_attempts=1,
        auto_remediate=True,
    )
    plan = res.get("remediation_plan")
    assert isinstance(plan, dict)
    assert plan["lesson_id"] == "PY001"
    assert "utils.py" in " ".join(plan.get("files_to_check") or [])
    diff_obj = plan.get("diff_objective")
    assert isinstance(diff_obj, dict)
    assert diff_obj["lesson_id"] == "PY001"
    assert "normalize_text" in " ".join(diff_obj.get("symbols") or [])
    assert "patch_template" in diff_obj
    assert "def normalize_text" in diff_obj["patch_template"]
    checklist = diff_obj.get("edge_case_checklist")
    assert isinstance(checklist, list)
    assert any("notimplementederror" in x.lower() for x in checklist)


def test_campaign_run_stops_after_completion(tmp_path: Path, monkeypatch):
    from brain.learning.ai_tutor import AITutor
    from brain.learning.python_mastery_campaign import campaign_run

    # Persistencia aislada
    monkeypatch.setattr("brain.learning.python_mastery_campaign.campaign_state_path", lambda repo_root=None: tmp_path / "state.json")

    # Evitar que campaign fuerce curriculum completo
    monkeypatch.setattr("brain.learning.python_mastery_campaign._ensure_python_curriculum", lambda tutor: None)

    # Currículum mini: PY001 -> PY002
    tutor = AITutor(tutor_type="disabled")
    tutor.curriculum = [
        {"lesson_id": "PY001", "name": "L1", "prerequisites": [], "learning_objectives": [], "tasks": []},
        {"lesson_id": "PY002", "name": "L2", "prerequisites": ["PY001"], "learning_objectives": [], "tasks": []},
    ]

    # Evaluador: siempre aprueba
    def fake_eval(*, lesson_id: str, repo_root, timeout_s: int = 180):
        return {"lesson_id": lesson_id, "score": 80, "passed": True, "knowledge_corrected": []}

    monkeypatch.setattr("brain.learning.python_mastery_evaluator.evaluate_python_mastery_lesson", fake_eval)
    monkeypatch.setattr("brain.learning.python_mastery_campaign.time.sleep", lambda s: None)

    loop = MagicMock()
    res = campaign_run(
        tutor=tutor,
        loop=loop,
        repo_root=tmp_path,
        reset=True,
        max_steps=10,
        max_seconds=5.0,
        step_max_attempts=1,
        step_backoff_s=0.0,
    )
    assert res["ok"] is True
    assert res["status"] == "completed"
    assert res["steps_run"] == 2

