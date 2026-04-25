from __future__ import annotations

from atlas_scanner.perception.sensors.camera import (
    Insta360StubProvider,
    OpenCvCameraProvider,
    StubCameraProvider,
    resolve_camera_provider,
)


def test_camera_provider_resolution_defaults_to_stub(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_CAMERA_PROVIDER", raising=False)
    provider = resolve_camera_provider()
    assert isinstance(provider, StubCameraProvider)


def test_camera_provider_resolution_opencv(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_CAMERA_PROVIDER", "opencv")
    monkeypatch.setenv("ATLAS_CAMERA_CAPTURE_ENABLED", "false")
    provider = resolve_camera_provider()
    assert isinstance(provider, OpenCvCameraProvider)


def test_camera_provider_resolution_insta360_stub(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_CAMERA_PROVIDER", "insta360")
    provider = resolve_camera_provider()
    assert isinstance(provider, Insta360StubProvider)
