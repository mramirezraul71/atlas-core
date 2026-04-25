from __future__ import annotations

from atlas_scanner.perception.sensors.camera import StubCameraProvider


def test_stub_camera_health_disabled() -> None:
    provider = StubCameraProvider(capture_enabled=False)
    snapshot = provider.health()
    assert snapshot.provider_ready is False
    assert snapshot.status == "disabled"


def test_stub_camera_capture_enabled_returns_metadata() -> None:
    provider = StubCameraProvider(capture_enabled=True)
    snapshot = provider.capture_snapshot()
    assert snapshot.provider_ready is True
    assert snapshot.status == "ready"
    assert snapshot.frame_width == 640
    assert snapshot.frame_height == 360
