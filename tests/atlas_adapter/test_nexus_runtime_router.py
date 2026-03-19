from pathlib import Path

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_adapter.routes.nexus_runtime as nexus_runtime
from atlas_adapter.routes.nexus_runtime import build_router


def test_nexus_runtime_router_exposes_expected_paths():
    router = build_router(Path("C:/repo"), Path("C:/repo/config/atlas.env"))
    paths = {route.path for route in router.routes}

    assert "/api/nexus/connection" in paths
    assert "/api/nexus/reconnect" in paths
    assert "/api/robot/status" in paths
    assert "/api/robot/reconnect" in paths
    assert "/api/cuerpo/reconnect" in paths
    assert "/api/robot/start-commands" in paths
    assert "/api/robot/log/tail" in paths
    assert "/api/nexus/log/tail" in paths
    assert "/nervous/services" in paths
    assert "/api/nerve/status" in paths
    assert "/api/feet/execute" in paths


def test_nexus_runtime_aux_routes_delegate_to_services(monkeypatch, tmp_path: Path):
    monkeypatch.setattr(
        nexus_runtime,
        "get_robot_log_tail",
        lambda base_dir, lines=200: {
            "ok": True,
            "path": str(base_dir / "logs" / "robot_backend.log"),
            "lines": int(lines),
            "text": "robot",
        },
    )
    monkeypatch.setattr(
        nexus_runtime,
        "get_nexus_log_tail",
        lambda base_dir, lines=200: {
            "ok": True,
            "path": str(base_dir / "logs" / "nexus_api.log"),
            "lines": int(lines),
            "text": "nexus",
        },
    )
    monkeypatch.setattr(
        nexus_runtime,
        "get_nerve_status",
        lambda: {
            "ok": True,
            "eyes": {"connected": True},
            "hands": {"local": True, "deps_ok": True},
            "feet": {"driver": "digital"},
        },
    )
    monkeypatch.setattr(
        nexus_runtime,
        "execute_feet_command",
        lambda command, payload=None: {
            "ok": True,
            "command": command,
            "payload": payload or {},
        },
    )

    app = FastAPI()
    app.include_router(build_router(tmp_path, tmp_path / "atlas.env"))
    client = TestClient(app)

    robot_log = client.get("/api/robot/log/tail", params={"lines": 12}).json()
    nexus_log = client.get("/api/nexus/log/tail", params={"lines": 7}).json()
    nervous = client.get("/nervous/services").json()
    nerve = client.get("/api/nerve/status").json()
    feet = client.post(
        "/api/feet/execute",
        json={"command": "open_url", "payload": {"url": "https://atlas.test"}},
    ).json()

    assert robot_log["text"] == "robot"
    assert robot_log["lines"] == 12
    assert nexus_log["text"] == "nexus"
    assert nexus_log["lines"] == 7
    assert nervous["eyes"]["connected"] is True
    assert nerve["feet"]["driver"] == "digital"
    assert feet["command"] == "open_url"
    assert feet["payload"]["url"] == "https://atlas.test"
