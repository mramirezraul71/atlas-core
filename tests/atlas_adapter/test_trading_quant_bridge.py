from __future__ import annotations

from pathlib import Path

from atlas_adapter.services.trading_quant_bridge import (
    call_quant_operation_cycle,
    capture_quant_vision_context,
    get_quant_bridge_status,
    run_quant_vision_cycle,
)


def test_capture_quant_vision_context_uses_gaze_and_capture(monkeypatch):
    monkeypatch.setattr(
        "atlas_adapter.services.trading_quant_bridge.look_at_screen",
        lambda x, y, zoom=None, calib_path=None, source="camera": {
            "ok": True,
            "screen_target": {"x": x, "y": y},
            "neck_pose": {"zoom": zoom or 1.0},
            "source": source,
            "calibration_path": calib_path,
        },
    )
    monkeypatch.setattr(
        "atlas_adapter.services.trading_quant_bridge.grasp",
        lambda target_id: {
            "ok": True,
            "target": target_id,
            "path": "C:/snapshots/quant.png",
            "resource_id": "file:C:/snapshots/quant.png",
        },
    )

    result = capture_quant_vision_context(
        capture_target="nexus:camera",
        screen_target={"x": 3200, "y": 1200, "zoom": 1.4},
        calib_path="C:/calib/screen.json",
        source="screen",
    )

    assert result["capture_ok"] is True
    assert result["capture_path"] == "C:/snapshots/quant.png"
    assert result["resource_id"] == "file:C:/snapshots/quant.png"
    assert result["gaze"]["screen_target"] == {"x": 3200.0, "y": 1200.0}
    assert result["gaze"]["source"] == "screen"


def test_call_quant_operation_cycle_uses_api_contract(monkeypatch):
    captured: dict[str, object] = {}

    def fake_json_post(url, payload, *, headers=None, timeout_sec=20):
        captured["url"] = url
        captured["payload"] = payload
        captured["headers"] = headers
        captured["timeout_sec"] = timeout_sec
        return True, 200, {"ok": True, "data": {"decision": "eligible"}}, ""

    monkeypatch.setattr(
        "atlas_adapter.services.trading_quant_bridge._json_post",
        fake_json_post,
    )

    result = call_quant_operation_cycle(
        order={"symbol": "SPY"},
        action="preview",
        capture_context=False,
        timeout_sec=9,
        api_base="http://127.0.0.1:8792",
        api_key="atlas-quant-local",
    )

    assert result["ok"] is True
    assert captured["url"] == "http://127.0.0.1:8792/api/v2/quant/operation/test-cycle"
    assert captured["payload"] == {
        "order": {"symbol": "SPY"},
        "action": "preview",
        "capture_context": False,
    }
    assert captured["headers"] == {"x-api-key": "atlas-quant-local"}
    assert captured["timeout_sec"] == 9


def test_run_quant_vision_cycle_combines_bridge_outputs(monkeypatch):
    monkeypatch.setattr(
        "atlas_adapter.services.trading_quant_bridge.capture_quant_vision_context",
        lambda **kwargs: {"capture_ok": True, "capture_path": "C:/snapshots/quant.png"},
    )
    monkeypatch.setattr(
        "atlas_adapter.services.trading_quant_bridge.call_quant_operation_cycle",
        lambda **kwargs: {"ok": True, "status_code": 200, "payload": {"ok": True}},
    )

    result = run_quant_vision_cycle(
        order={"symbol": "QQQ"},
        action="evaluate",
        include_vision=True,
    )

    assert result["ok"] is True
    assert result["vision_snapshot"]["capture_path"] == "C:/snapshots/quant.png"
    assert result["quant"]["status_code"] == 200


def test_get_quant_bridge_status_exposes_prompt_path(tmp_path: Path):
    status = get_quant_bridge_status(repo_root=tmp_path)

    assert status["ok"] is True
    assert status["bridge_endpoints"]["vision_cycle"] == "/api/trading/quant/vision-cycle"
    assert status["handoff_prompt_path"].endswith("prompts\\quant_native_camera_integration_handoff.md")
