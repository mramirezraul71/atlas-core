from __future__ import annotations

import queue
import threading
import time
from typing import Any

import cv2
import mediapipe as mp
import numpy as np

try:
    from .config import VisionSensorConfig
    from .event_publisher import EventPublisher
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig
    from event_publisher import EventPublisher


class GestureDetector:
    def __init__(self, config: VisionSensorConfig, publisher: EventPublisher, logger: Any) -> None:
        self.config = config
        self.publisher = publisher
        self.logger = logger
        self.queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=config.queue_maxsize)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._hands = mp.solutions.hands.Hands(
            max_num_hands=2,
            min_detection_confidence=config.mediapipe_min_confidence,
            min_tracking_confidence=config.mediapipe_min_confidence,
        )
        self._active_since: float | None = None
        self._last_emit_ts = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, name="vision-gesture-detector", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._hands.close()

    def submit(self, frame: np.ndarray) -> None:
        try:
            self.queue.put_nowait(frame.copy())
        except queue.Full:
            pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process(frame)
            except Exception:
                self._active_since = None

    def _process(self, frame: np.ndarray) -> None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb)
        now = time.monotonic()
        detected = False
        confidence = 0.0

        if result.multi_hand_landmarks and result.multi_handedness:
            for hand_landmarks, handedness in zip(result.multi_hand_landmarks, result.multi_handedness):
                label = str(handedness.classification[0].label or "")
                score = float(handedness.classification[0].score or 0.0)
                if label.lower() != "right" or score < self.config.emergency_min_confidence:
                    continue
                if self._is_closed_fist(hand_landmarks):
                    detected = True
                    confidence = score
                    break

        if not detected or confidence < self.config.emergency_min_confidence:
            self._active_since = None
            return
        if self._active_since is None:
            self._active_since = now
            return
        if now - self._active_since < self.config.emergency_hold_sec:
            return
        if now - self._last_emit_ts < 5.0:
            return
        self._last_emit_ts = now
        self.publisher.emit(
            "emergency_close",
            severity="CRITICAL",
            confidence=confidence,
            payload={
                "gesture": "right_closed_fist",
                "hold_sec": round(now - self._active_since, 3),
            },
            sensor_state="ACTIVE",
        )
        self._active_since = None

    @staticmethod
    def _is_closed_fist(hand_landmarks: Any) -> bool:
        lm = hand_landmarks.landmark
        fingers_curled = [
            lm[8].y > lm[6].y,
            lm[12].y > lm[10].y,
            lm[16].y > lm[14].y,
            lm[20].y > lm[18].y,
        ]
        thumb_folded = abs(lm[4].x - lm[2].x) < 0.12 and abs(lm[4].y - lm[2].y) < 0.12
        return all(fingers_curled) and thumb_folded
