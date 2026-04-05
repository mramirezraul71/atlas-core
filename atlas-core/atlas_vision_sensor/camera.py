from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

try:
    from .config import VisionSensorConfig
    from .event_publisher import EventPublisher
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig
    from event_publisher import EventPublisher


@dataclass(slots=True)
class CameraStatus:
    connected: bool
    camera_index: int | None
    degraded: bool
    retries: int


class CameraService:
    def __init__(self, config: VisionSensorConfig, publisher: EventPublisher, logger: Any) -> None:
        self.config = config
        self.publisher = publisher
        self.logger = logger
        self._cap: cv2.VideoCapture | None = None
        self._camera_index: int | None = None
        self._lock = threading.Lock()
        self._reader_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._frame: np.ndarray | None = None
        self._connected = False
        self._degraded = False
        self._retries = 0

    @property
    def camera_index(self) -> int | None:
        return self._camera_index

    def start(self) -> CameraStatus:
        connected = self._connect_with_retry()
        if connected:
            self._reader_thread = threading.Thread(target=self._reader_loop, name="vision-camera-reader", daemon=True)
            self._reader_thread.start()
        return self.status()

    def stop(self) -> None:
        self._stop.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None
        self._connected = False

    def status(self) -> CameraStatus:
        return CameraStatus(
            connected=self._connected,
            camera_index=self._camera_index,
            degraded=self._degraded,
            retries=self._retries,
        )

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def reconnect(self) -> bool:
        self.stop()
        self._stop.clear()
        connected = self._connect_with_retry()
        if connected:
            self._reader_thread = threading.Thread(target=self._reader_loop, name="vision-camera-reader", daemon=True)
            self._reader_thread.start()
        return connected

    def _connect_with_retry(self) -> bool:
        for attempt in range(1, 4):
            self._retries = attempt
            if self._detect_and_open():
                self._connected = True
                self._degraded = False
                return True
            time.sleep(min(2 ** (attempt - 1), 8))
        self._connected = False
        self._degraded = True
        self.publisher.emit(
            "camera_offline",
            severity="ERROR",
            confidence=1.0,
            payload={"camera_indexes": list(self.config.camera_indexes), "retries": self._retries},
            sensor_state="DEGRADED",
        )
        return False

    def _detect_and_open(self) -> bool:
        for index in self.config.camera_indexes:
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            if not cap or not cap.isOpened():
                if cap:
                    cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.capture_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.capture_height)
            ok, frame = cap.read()
            if ok and frame is not None and getattr(frame, "size", 0) > 0:
                with self._lock:
                    if self._cap is not None:
                        self._cap.release()
                    self._cap = cap
                    self._camera_index = index
                    self._frame = frame.copy()
                return True
            cap.release()
        return False

    def _reader_loop(self) -> None:
        interval = 1.0 / max(1, self.config.process_fps)
        while not self._stop.is_set():
            with self._lock:
                cap = self._cap
            if cap is None or not cap.isOpened():
                self._connected = False
                self._degraded = True
                time.sleep(interval)
                continue
            ok, frame = cap.read()
            if ok and frame is not None and getattr(frame, "size", 0) > 0:
                with self._lock:
                    self._frame = frame.copy()
                self._connected = True
            else:
                self._connected = False
            time.sleep(interval)
