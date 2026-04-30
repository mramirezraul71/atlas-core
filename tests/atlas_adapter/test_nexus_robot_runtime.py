from pathlib import Path

from atlas_adapter.services.nexus_robot_runtime import (
    get_nexus_connection_payload,
    get_robot_log_tail,
    get_robot_start_commands,
    reconnect_cuerpo,
    reconnect_robot,
    tail_text_file,
)


def test_get_nexus_connection_payload_shape():
    payload = get_nexus_connection_payload()

    assert payload["ok"] is True
    assert payload["connected"] is True
    assert payload["active"] is True
    assert "last_check_ts" in payload
    assert payload["last_error"] == ""


def test_reconnect_robot_reports_missing_script(tmp_path: Path, monkeypatch):
    monkeypatch.delenv("NEXUS_ROBOT_PATH", raising=False)

    result = reconnect_robot(tmp_path, tmp_path / "atlas.env")

    assert result["ok"] is False
    assert result["connected"] is False
    assert result["message"] == "Script no encontrado."


def test_reconnect_cuerpo_reports_missing_nexus_dir(tmp_path: Path, monkeypatch):
    script = tmp_path / "scripts" / "start_nexus_services.py"
    script.parent.mkdir(parents=True)
    script.write_text("print('stub')", encoding="utf-8")
    monkeypatch.setenv("NEXUS_ATLAS_PATH", str(tmp_path / "missing-nexus"))
    monkeypatch.setenv("NEXUS_ROBOT_PATH", str(tmp_path / "robot"))
    (tmp_path / "robot").mkdir()

    result = reconnect_cuerpo(tmp_path, tmp_path / "atlas.env")

    assert result["ok"] is False
    assert result["started"] is False
    assert "Carpeta NEXUS no existe" in result["message"]


def test_robot_start_commands_uses_repo_defaults(tmp_path: Path, monkeypatch):
    monkeypatch.delenv("NEXUS_ATLAS_PATH", raising=False)
    monkeypatch.delenv("NEXUS_ROBOT_PATH", raising=False)
    monkeypatch.setenv("PYTHON", "python-test")

    result = get_robot_start_commands(tmp_path, tmp_path / "atlas.env")

    assert result["ok"] is True
    assert result["robot_path"].endswith("nexus\\atlas_nexus_robot\\backend")
    assert result["nexus_path"].endswith("nexus\\atlas_nexus")
    assert "main.py" in result["commands"]["robot"]
    assert "nexus.py --mode api" in result["commands"]["nexus"]


def test_tail_text_file_returns_last_lines(tmp_path: Path):
    log_file = tmp_path / "tail.log"
    log_file.write_text("a\nb\nc\nd\n", encoding="utf-8")

    result = tail_text_file(log_file, lines=2)

    assert result == "c\nd"


def test_get_robot_log_tail_uses_logs_dir(tmp_path: Path):
    logs_dir = tmp_path / "logs"
    logs_dir.mkdir()
    (logs_dir / "robot_backend.log").write_text("uno\ndos\ntres\n", encoding="utf-8")

    result = get_robot_log_tail(tmp_path, lines=2)

    assert result["ok"] is True
    assert result["path"].endswith("logs\\robot_backend.log")
    assert result["text"] == "dos\ntres"
