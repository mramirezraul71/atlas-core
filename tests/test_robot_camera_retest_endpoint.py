from __future__ import annotations

import importlib.util
import uuid
from pathlib import Path

from fastapi.testclient import TestClient


def _load_robot_backend_main():
    root = Path(__file__).resolve().parents[1]
    module_path = root / "nexus" / "atlas_nexus_robot" / "backend" / "main.py"
    module_name = f"atlas_robot_backend_main_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.app.router.on_startup.clear()
    module.app.router.on_shutdown.clear()
    return module


def test_camera_retest_marks_camera_on_when_available(monkeypatch):
    robot_main = _load_robot_backend_main()
    robot_main.robot_status["modules"]["camera"] = False
    robot_main.robot_status["modules"]["sensors"] = False

    def _fake_camera_eval(device_index=None, try_start_manager=True):
        return {
            "available": True,
            "device_index": device_index if device_index is not None else 1,
            "error": None,
            "manager_running": True,
        }

    monkeypatch.setattr(robot_main, "evaluate_camera_status", _fake_camera_eval)

    client = TestClient(robot_main.app)
    response = client.post("/modules/camera/retest", json={"device_index": 1})
    payload = response.json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["camera_available"] is True
    assert payload["device_index"] == 1
    assert payload["error"] is None
    assert robot_main.robot_status["modules"]["camera"] is True
    assert robot_main.robot_status["modules"]["sensors"] is True


def test_camera_retest_keeps_camera_off_when_unavailable(monkeypatch):
    robot_main = _load_robot_backend_main()
    robot_main.robot_status["modules"]["camera"] = False
    robot_main.robot_status["modules"]["sensors"] = False

    def _fake_camera_eval(device_index=None, try_start_manager=True):
        return {
            "available": False,
            "device_index": device_index if device_index is not None else 0,
            "error": "cannot_open_device_0",
            "manager_running": False,
        }

    monkeypatch.setattr(robot_main, "evaluate_camera_status", _fake_camera_eval)

    client = TestClient(robot_main.app)
    response = client.post("/modules/camera/retest", json={"device_index": 0})
    payload = response.json()

    assert response.status_code == 200
    assert payload["ok"] is False
    assert payload["camera_available"] is False
    assert payload["device_index"] == 0
    assert payload["error"] == "cannot_open_device_0"
    assert robot_main.robot_status["modules"]["camera"] is False
    assert robot_main.robot_status["modules"]["sensors"] is False
