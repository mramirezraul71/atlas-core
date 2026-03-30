from __future__ import annotations

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
