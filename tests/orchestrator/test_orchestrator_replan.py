from __future__ import annotations


def test_run_goal_replans_on_failure(monkeypatch):
    import modules.humanoid.orchestrator.orchestrator as orch

    # Avoid sqlite writes in unit test
    monkeypatch.setattr(orch, "save_task", lambda *a, **k: None)

    calls = {"decompose": 0, "exec": []}

    def fake_decompose(goal: str, fast: bool = True):
        calls["decompose"] += 1
        # First plan fails, second plan succeeds
        if "Contexto de replanificación" in goal:
            return {"ok": True, "steps": [{"id": "step_1", "description": "ok_step", "definition_of_done": None, "status": "pending"}], "raw_steps": ["ok_step"], "error": None}
        return {"ok": True, "steps": [{"id": "step_1", "description": "fail_step", "definition_of_done": None, "status": "pending"}], "raw_steps": ["fail_step"], "error": None}

    def fake_exec(step, task_id):
        desc = step.get("description")
        calls["exec"].append(desc)
        if desc == "fail_step":
            return {"step_id": step.get("id"), "status": "failed", "error": "boom", "result": {"ok": False}, "artifacts": []}
        return {"step_id": step.get("id"), "status": "success", "error": None, "result": {"ok": True}, "artifacts": []}

    # Disable healing to isolate replan behavior
    monkeypatch.setattr(orch, "_decompose", fake_decompose)
    monkeypatch.setattr(orch, "_execute_step_internal", fake_exec)
    monkeypatch.setattr(orch, "_try_heal_step", lambda *a, **k: {"attempted": False, "success": False, "message": "skip"})

    out = orch.run_goal("objetivo de prueba", mode="execute", fast=True)
    assert out["ok"] is True
    assert out.get("replans_used", 0) >= 1
    assert "fail_step" in calls["exec"]
    assert "ok_step" in calls["exec"]

