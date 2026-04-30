from __future__ import annotations

import numpy as np

from geometry_detector import GeometryDetector


def test_geometry_detector_handles_blank_frame() -> None:
    detector = GeometryDetector()
    frame = np.zeros((720, 1280), dtype=np.uint8)
    result = detector.detect(frame)
    assert result.confidence == 0.0 or result.payload["line_count"] >= 0
