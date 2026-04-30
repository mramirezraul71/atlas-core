from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from .config import VisionSensorConfig
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig


@dataclass(slots=True)
class BufferedFrame:
    ts: float
    frame: np.ndarray


class ForensicRecorder:
    def __init__(self, config: VisionSensorConfig, logger: Any) -> None:
        self.config = config
        self.logger = logger
        maxlen = max(10, config.forensic_buffer_sec * config.forensic_video_fps)
        self._buffer: deque[BufferedFrame] = deque(maxlen=maxlen)
        self._last_capture_ts = 0.0

    def observe_frame(self, frame: np.ndarray | None) -> None:
        if frame is None or getattr(frame, "size", 0) == 0:
            return
        self._buffer.append(BufferedFrame(ts=time.time(), frame=frame.copy()))

    def maybe_capture(
        self,
        *,
        event_type: str,
        severity: str,
        payload: dict[str, Any],
        sensor_state: str,
        current_frame: np.ndarray | None = None,
    ) -> dict[str, Any] | None:
        if not self._should_capture(event_type=event_type, severity=severity, payload=payload):
            return None
        now = time.time()
        if now - self._last_capture_ts < 5.0:
            return None
        self._last_capture_ts = now
        artifact_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(now))
        artifact_dir = self.config.forensic_dir / artifact_id
        artifact_dir.mkdir(parents=True, exist_ok=True)

        frames = list(self._buffer)
        if current_frame is not None and getattr(current_frame, "size", 0) > 0:
            frames.append(BufferedFrame(ts=now, frame=current_frame.copy()))

        metadata = {
            "artifact_id": artifact_id,
            "event_type": event_type,
            "severity": severity,
            "sensor_state": sensor_state,
            "buffered_frames": len(frames),
            "payload": payload,
        }
        video_path = artifact_dir / "context.avi"
        metadata_path = artifact_dir / "context.json"
        metadata_path.write_text(json.dumps(metadata, ensure_ascii=True, indent=2), encoding="utf-8")
        saved = self._write_video(video_path, frames)
        if not saved and frames:
            fallback_path = artifact_dir / "last_frame.jpg"
            cv2.imwrite(str(fallback_path), frames[-1].frame)
            metadata["fallback_frame"] = str(fallback_path)
            metadata_path.write_text(json.dumps(metadata, ensure_ascii=True, indent=2), encoding="utf-8")

        self._enforce_retention()
        return {
            "artifact_id": artifact_id,
            "artifact_dir": str(artifact_dir),
            "video_path": str(video_path) if saved else None,
            "metadata_path": str(metadata_path),
            "saved": saved,
        }

    @staticmethod
    def _should_capture(*, event_type: str, severity: str, payload: dict[str, Any]) -> bool:
        if severity.upper() in {"CRITICAL", "HIGH"}:
            return True
        if event_type in {"emergency_close", "security_event"}:
            return True
        if event_type == "visual_signal":
            action = str(payload.get("action") or "")
            contradictions = payload.get("contradictions") or []
            if action in {"degrade", "block"} and contradictions:
                return True
        text = json.dumps(payload, ensure_ascii=True).lower()
        return any(token in text for token in ("slippage", "fill", "stop_loss", "reconciliation"))

    def _write_video(self, target: Path, frames: list[BufferedFrame]) -> bool:
        if not frames:
            return False
        height, width = frames[0].frame.shape[:2]
        writer = cv2.VideoWriter(
            str(target),
            cv2.VideoWriter_fourcc(*"MJPG"),
            float(self.config.forensic_video_fps),
            (int(width), int(height)),
        )
        if not writer.isOpened():
            return False
        try:
            for item in frames:
                frame = item.frame
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                writer.write(frame)
        finally:
            writer.release()
        return True

    def _enforce_retention(self) -> None:
        artifacts = sorted(
            [path for path in self.config.forensic_dir.iterdir() if path.is_dir()],
            key=lambda item: item.stat().st_mtime,
            reverse=True,
        )
        for extra in artifacts[self.config.forensic_max_artifacts :]:
            for child in extra.glob("**/*"):
                if child.is_file():
                    child.unlink(missing_ok=True)
            for child in sorted(extra.glob("**/*"), reverse=True):
                if child.is_dir():
                    child.rmdir()
            extra.rmdir()
