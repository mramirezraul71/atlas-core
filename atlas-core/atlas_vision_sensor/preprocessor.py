from __future__ import annotations

import cv2
import numpy as np

try:
    from .config import VisionSensorConfig
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig


class FramePreprocessor:
    def __init__(self, config: VisionSensorConfig) -> None:
        self.config = config

    def process(self, frame: np.ndarray | None) -> np.ndarray | None:
        if frame is None or getattr(frame, "size", 0) == 0:
            return None
        resized = cv2.resize(frame, (self.config.process_width, self.config.process_height), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        return normalized
