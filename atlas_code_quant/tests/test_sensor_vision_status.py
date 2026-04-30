from __future__ import annotations

from unittest.mock import Mock

from operations.sensor_vision import SensorVisionService


def test_sensor_vision_fast_status_skips_expensive_probes(tmp_path, monkeypatch):
    service = SensorVisionService(state_path=tmp_path / "sensor_vision_state.json")
    state = service._load()
    state["provider"] = "direct_nexus"
    service._save(state)

    def _boom(*args, **kwargs):
        raise AssertionError("fast status should not call expensive vision probes")

    monkeypatch.setattr(service, "_direct_nexus_available", _boom)
    monkeypatch.setattr(service, "_atlas_push_bridge_status", _boom)
    monkeypatch.setattr(service, "_insta360_available", _boom)

    payload = service.status(fast=True)

    assert payload["provider"] == "direct_nexus"
    assert payload["status_mode"] == "fast_cached"
    assert payload["provider_ready"] is True


def test_sensor_vision_full_status_keeps_probe_behavior(tmp_path, monkeypatch):
    service = SensorVisionService(state_path=tmp_path / "sensor_vision_state.json")
    state = service._load()
    state["provider"] = "direct_nexus"
    service._save(state)

    calls = {"direct": 0}

    def _direct_ok():
        calls["direct"] += 1
        return True

    monkeypatch.setattr(service, "_direct_nexus_available", _direct_ok)

    payload = service.status()

    assert payload["status_mode"] == "full"
    assert calls["direct"] >= 1
    assert payload["provider_ready"] is True


def test_status_for_gate_skips_insta360_probe_when_direct_nexus(tmp_path, monkeypatch):
    service = SensorVisionService(state_path=tmp_path / "sensor_vision_state.json")
    state = service._load()
    state["provider"] = "direct_nexus"
    service._save(state)

    def _no_insta(*args, **kwargs):
        raise AssertionError("gate no debe sondear Insta360 si el proveedor es direct_nexus")

    monkeypatch.setattr(service, "_insta360_available", _no_insta)
    monkeypatch.setattr(service, "_direct_nexus_available", lambda **kw: True)

    payload = service.status_for_gate()

    assert payload["status_mode"] == "gate"
    assert payload["provider_ready"] is True
    assert payload["insta360_available"] is None


def test_status_for_gate_push_bridge_uses_short_request_timeout(tmp_path, monkeypatch):
    service = SensorVisionService(state_path=tmp_path / "sensor_vision_state.json")
    state = service._load()
    state["provider"] = "atlas_push_bridge"
    service._save(state)

    mock_rj = Mock(return_value=(True, 200, {"ok": True}, ""))
    service._request_json = mock_rj  # type: ignore[method-assign]
    monkeypatch.setattr(service, "_direct_nexus_available", lambda **kw: True)

    service.status_for_gate(push_bridge_timeout_sec=2)

    assert mock_rj.called
    _args, kwargs = mock_rj.call_args
    assert kwargs.get("timeout_sec") == 2


def test_sensor_vision_normalizes_insta360_pending_alias(tmp_path):
    service = SensorVisionService(state_path=tmp_path / "sensor_vision_state.json")

    updated = service.update(provider="insta360_pending")
    payload = service.status(fast=True)

    assert updated["provider"] == "insta360"
    assert payload["provider"] == "insta360"
    assert "insta360" in payload["supported_modes"]
