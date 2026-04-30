from __future__ import annotations

import queue
import threading
import time
from typing import Any

import numpy as np

try:
    from .camera import CameraService
    from .config import VisionSensorConfig
    from .event_publisher import EventPublisher
except ImportError:  # pragma: no cover
    from camera import CameraService
    from config import VisionSensorConfig
    from event_publisher import EventPublisher


class SensorWatchdog:
    def __init__(self, config: VisionSensorConfig, camera: CameraService, publisher: EventPublisher, logger: Any) -> None:
        self.config = config
        self.camera = camera
        self.publisher = publisher
        self.logger = logger
        self.queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=config.queue_maxsize)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_frame: np.ndarray | None = None
        self._last_change_ts = time.monotonic()
        self._last_heartbeat_ts = 0.0
        self._mode = "ACTIVE"

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, name="vision-watchdog", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def submit(self, frame: np.ndarray) -> None:
        try:
            self.queue.put_nowait(frame.copy())
        except queue.Full:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            self._consume_latest_frame()
            if now - self._last_heartbeat_ts >= self.config.heartbeat_interval_sec:
                self.publisher.emit(
                    "heartbeat",
                    severity="INFO",
                    confidence=1.0,
                    payload={"mode": self._mode},
                    sensor_state=self._mode,
                )
                self._last_heartbeat_ts = now
            self._evaluate_state(now)
            time.sleep(self.config.watchdog_interval_sec)

    def _consume_latest_frame(self) -> None:
        latest: np.ndarray | None = None
        while True:
            try:
                latest = self.queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return
        if self._last_frame is None:
            self._last_frame = latest
            self._last_change_ts = time.monotonic()
            return
        diff = float(np.mean(np.abs(latest.astype(np.float32) - self._last_frame.astype(np.float32))))
        if diff >= 1.25:
            self._last_change_ts = time.monotonic()
            self._last_frame = latest

    def _evaluate_state(self, now: float) -> None:
        frozen_for = now - self._last_change_ts
        cam_status = self.camera.status()
        if not cam_status.connected:
            self._mode = "OFFLINE"
            self.publisher.emit(
                "sensor_degraded",
                severity="ERROR",
                confidence=1.0,
                payload={"cause": "camera_offline", "severity": "HIGH", "frozen_for_sec": round(frozen_for, 2)},
                sensor_state=self._mode,
            )
            return
        if frozen_for < self.config.freeze_timeout_sec:
            self._mode = "ACTIVE"
            return

        cause = self._classify_freeze(frozen_for)
        self._mode = "DEGRADED"
        self.publisher.emit(
            "sensor_degraded",
            severity="WARNING" if cause == "false_positive" else "ERROR",
            confidence=0.8,
            payload={"cause": cause, "severity": "MEDIUM" if cause == "false_positive" else "HIGH"},
            sensor_state=self._mode,
        )
        reconnect_ok = False
        for _ in range(3):
            reconnect_ok = self.camera.reconnect()
            if reconnect_ok:
                self._last_change_ts = time.monotonic()
                self._mode = "ACTIVE"
                return
        self._mode = "DEGRADED"
        self.publisher.emit(
            "sensor_standby",
            severity="CRITICAL",
            confidence=1.0,
            payload={"cause": cause, "next_action": "await_orchestrator_reset"},
            sensor_state="DEGRADED",
        )

    def _classify_freeze(self, frozen_for: float) -> str:
        status = self.camera.status()
        if not status.connected:
            return "camera_offline"
        if self._last_frame is None:
            return "driver_hang"
        variance = float(np.var(self._last_frame))
        if variance < 12.0:
            return "false_positive"
        if frozen_for >= self.config.freeze_timeout_sec * 1.5:
            return "driver_hang"
        return "ui_freeze"
