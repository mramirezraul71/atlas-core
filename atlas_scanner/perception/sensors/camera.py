from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import os
from typing import Any, Protocol


@dataclass(frozen=True)
class CameraSnapshot:
    provider: str
    source_id: str
    timestamp: str
    provider_ready: bool
    status: str
    message: str | None
    frame_width: int | None = None
    frame_height: int | None = None
    presence_score: float | None = None
    activity_level: float | None = None


class CameraSensorProvider(Protocol):
    def health(self) -> CameraSnapshot:
        ...

    def capture_snapshot(self) -> CameraSnapshot:
        ...


@dataclass
class StubCameraProvider:
    capture_enabled: bool = False
    source_id: str = "camera:stub"

    def health(self) -> CameraSnapshot:
        now = datetime.now(timezone.utc).isoformat()
        if not self.capture_enabled:
            return CameraSnapshot(
                provider="stub",
                source_id=self.source_id,
                timestamp=now,
                provider_ready=False,
                status="disabled",
                message="camera_capture_disabled",
            )
        return CameraSnapshot(
            provider="stub",
            source_id=self.source_id,
            timestamp=now,
            provider_ready=True,
            status="ready",
            message="stub_camera_ready",
            presence_score=0.1,
            activity_level=0.0,
        )

    def capture_snapshot(self) -> CameraSnapshot:
        snapshot = self.health()
        if not self.capture_enabled:
            return snapshot
        return CameraSnapshot(
            provider="stub",
            source_id=self.source_id,
            timestamp=snapshot.timestamp,
            provider_ready=True,
            status="ready",
            message="stub_capture",
            frame_width=640,
            frame_height=360,
            presence_score=0.2,
            activity_level=0.15,
        )


@dataclass
class OpenCvCameraProvider:
    device_index: int = 0
    capture_enabled: bool = False
    source_id: str = "camera:opencv"

    def health(self) -> CameraSnapshot:
        now = datetime.now(timezone.utc).isoformat()
        if not self.capture_enabled:
            return CameraSnapshot(
                provider="opencv",
                source_id=self.source_id,
                timestamp=now,
                provider_ready=False,
                status="disabled",
                message="camera_capture_disabled",
            )
        try:
            cv2 = importlib.import_module("cv2")
        except Exception:
            return CameraSnapshot(
                provider="opencv",
                source_id=self.source_id,
                timestamp=now,
                provider_ready=False,
                status="degraded",
                message="opencv_module_unavailable",
            )
        cap = cv2.VideoCapture(self.device_index)
        try:
            opened = bool(cap.isOpened())
            if not opened:
                return CameraSnapshot(
                    provider="opencv",
                    source_id=self.source_id,
                    timestamp=now,
                    provider_ready=False,
                    status="degraded",
                    message="camera_device_unavailable",
                )
            return CameraSnapshot(
                provider="opencv",
                source_id=self.source_id,
                timestamp=now,
                provider_ready=True,
                status="ready",
                message="camera_device_ready",
            )
        finally:
            cap.release()

    def capture_snapshot(self) -> CameraSnapshot:
        health = self.health()
        if not health.provider_ready:
            return health
        cv2 = importlib.import_module("cv2")
        cap = cv2.VideoCapture(self.device_index)
        try:
            ok, frame = cap.read()
            if not ok or frame is None:
                return CameraSnapshot(
                    provider="opencv",
                    source_id=self.source_id,
                    timestamp=datetime.now(timezone.utc).isoformat(),
                    provider_ready=False,
                    status="degraded",
                    message="camera_capture_failed",
                )
            height, width = frame.shape[:2]
            # Base multimodal simple: proxy de actividad por magnitud media.
            activity_level = float(abs(frame.astype("float32").mean()) / 255.0)
            presence_score = min(1.0, activity_level * 1.25)
            return CameraSnapshot(
                provider="opencv",
                source_id=self.source_id,
                timestamp=datetime.now(timezone.utc).isoformat(),
                provider_ready=True,
                status="ready",
                message="camera_capture_ok",
                frame_width=int(width),
                frame_height=int(height),
                presence_score=round(presence_score, 4),
                activity_level=round(activity_level, 4),
            )
        finally:
            cap.release()


@dataclass
class Insta360StubProvider:
    capture_enabled: bool = False
    source_id: str = "camera:insta360"

    def health(self) -> CameraSnapshot:
        return CameraSnapshot(
            provider="insta360",
            source_id=self.source_id,
            timestamp=datetime.now(timezone.utc).isoformat(),
            provider_ready=False,
            status="degraded",
            message="insta360_provider_stub_only",
        )

    def capture_snapshot(self) -> CameraSnapshot:
        return self.health()


def resolve_camera_provider() -> CameraSensorProvider:
    provider_name = (os.getenv("ATLAS_CAMERA_PROVIDER", "stub").strip().lower() or "stub")
    capture_enabled = os.getenv("ATLAS_CAMERA_CAPTURE_ENABLED", "false").strip().lower() in {"1", "true", "yes"}
    device_index = int(os.getenv("ATLAS_CAMERA_DEVICE_INDEX", "0"))
    source_id = os.getenv("ATLAS_CAMERA_SOURCE_ID", f"camera:{provider_name}").strip() or f"camera:{provider_name}"
    if provider_name == "opencv":
        return OpenCvCameraProvider(device_index=device_index, capture_enabled=capture_enabled, source_id=source_id)
    if provider_name == "insta360":
        return Insta360StubProvider(capture_enabled=capture_enabled, source_id=source_id)
    return StubCameraProvider(capture_enabled=capture_enabled, source_id=source_id)


def to_camera_context(snapshot: CameraSnapshot) -> dict[str, Any]:
    return {
        "provider": snapshot.provider,
        "source_id": snapshot.source_id,
        "provider_ready": snapshot.provider_ready,
        "status": snapshot.status,
        "message": snapshot.message,
        "last_capture": snapshot.timestamp,
        "presence_score": snapshot.presence_score,
        "activity_level": snapshot.activity_level,
        "frame_width": snapshot.frame_width,
        "frame_height": snapshot.frame_height,
    }
