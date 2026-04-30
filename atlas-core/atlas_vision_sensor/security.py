from __future__ import annotations

import queue
import threading
import time
from typing import Any

import cv2
import numpy as np

try:
    from .config import VisionSensorConfig
    from .event_publisher import EventPublisher
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig
    from event_publisher import EventPublisher


class SecurityMonitor:
    def __init__(self, config: VisionSensorConfig, publisher: EventPublisher, logger: Any) -> None:
        self.config = config
        self.publisher = publisher
        self.logger = logger
        self.queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=config.queue_maxsize)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._intrusion_active = False
        self._lock = threading.Lock()
        self._last_emit_ts = 0.0
        self._emit_cooldown_sec = 10.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, name="vision-security-monitor", daemon=True)
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

    def apply_privacy_mask(self, frame: np.ndarray | None) -> np.ndarray | None:
        if frame is None or not self.config.privacy_mask_enabled:
            return frame
        with self._lock:
            intrusion_active = self._intrusion_active
        if not intrusion_active:
            return frame
        masked = frame.copy()
        h, w = masked.shape[:2]
        x0, y0 = int(w * 0.15), int(h * 0.12)
        x1, y1 = int(w * 0.88), int(h * 0.86)
        roi = masked[y0:y1, x0:x1]
        if roi.size:
            masked[y0:y1, x0:x1] = cv2.GaussianBlur(roi, (51, 51), 0)
        return masked

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process(frame)
            except Exception:
                continue

    def _process(self, frame: np.ndarray) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects, weights = self._hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
        best_weight = max((float(w) for w in weights), default=0.0)
        detected = len(rects) > 0 and best_weight >= self.config.people_confidence_threshold
        with self._lock:
            self._intrusion_active = detected
        if detected:
            now = time.monotonic()
            if now - self._last_emit_ts < self._emit_cooldown_sec:
                return
            self._last_emit_ts = now
            severity = "HIGH" if len(rects) > 1 or best_weight > 1.0 else "MEDIUM"
            self.publisher.emit(
                "security_event",
                severity=severity,
                confidence=min(1.0, max(0.0, best_weight)),
                payload={
                    "cause": "unauthorized_presence_detected",
                    "people_count": int(len(rects)),
                    "privacy_mask": True,
                },
                sensor_state="ACTIVE",
            )
