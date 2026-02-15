from __future__ import annotations

from pathlib import Path


def test_cursor_executor_needs_human_for_unmapped_step(tmp_path: Path, monkeypatch):
    from modules.humanoid.cursor.executor import CursorExecutor

    # aislar repo_root para no escribir en snapshots reales
    ex = CursorExecutor(repo_root=tmp_path)

    # monkeypatch de shell para evitar ejecutar comandos reales
    monkeypatch.setattr(ex.shell, "run", lambda cmd, cwd=None, timeout_sec=60, actor=None: {"ok": True, "stdout": "ok", "stderr": "", "returncode": 0, "error": None})

    steps = [{"description": "Haz magia con mi módulo X", "status": "pending"}]
    out = ex.execute_steps(steps, goal="prueba")

    assert "artifacts" in out
    assert Path(out["artifacts"]["terminal_log_path"]).exists()
    assert Path(out["artifacts"]["script_path"]).exists()
    assert Path(out["artifacts"]["run_json_path"]).exists()
    assert out["executed"][0]["action"] == "needs_human"
    assert out["executed"][0]["ok"] is False
    assert steps[0]["status"] == "needs_human"


def test_cursor_executor_runs_status_without_shell(tmp_path: Path):
    from modules.humanoid.cursor.executor import CursorExecutor

    ex = CursorExecutor(repo_root=tmp_path)
    out = ex.execute_steps([{"description": "Verificar estado del sistema: /status", "status": "pending"}], goal="status")
    assert out["executed"][0]["action"] == "status"
    # puede fallar si el command_router no existe en este contexto, pero en repo debe existir
    assert "details" in out["executed"][0]

