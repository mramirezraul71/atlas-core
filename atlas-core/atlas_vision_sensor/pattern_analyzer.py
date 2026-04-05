from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

try:
    from atlas_code_quant.vision.chart_ocr import ChartOCR
except Exception:  # pragma: no cover
    ChartOCR = None  # type: ignore[assignment]


@dataclass(slots=True)
class PatternResult:
    pattern: str
    confidence: float
    payload: dict[str, Any]


class PatternAnalyzer:
    def __init__(self) -> None:
        self._ocr = ChartOCR() if ChartOCR is not None else None

    def analyze(
        self,
        processed_frame: np.ndarray | None,
        *,
        raw_frame: np.ndarray | None = None,
    ) -> PatternResult | None:
        if processed_frame is None or getattr(processed_frame, "size", 0) == 0:
            return None
        h, w = processed_frame.shape[:2]
        edges = cv2.Canny(processed_frame, 80, 160)
        edge_density = float(np.count_nonzero(edges)) / float(edges.size)
        column_mean = processed_frame.mean(axis=0)
        slope = float(np.polyfit(np.arange(len(column_mean)), column_mean, 1)[0]) if len(column_mean) > 2 else 0.0
        volatility = float(np.std(column_mean))
        ocr_payload: dict[str, Any] = {}
        chart_bias = "unknown"
        ocr_pattern = ""
        if raw_frame is not None and self._ocr is not None:
            try:
                ocr_result = self._ocr.analyze(raw_frame)
                chart_bias = str(ocr_result.chart_color or "unknown")
                ocr_pattern = str(ocr_result.pattern_detected or "")
                ocr_payload = {
                    "chart_bias": chart_bias,
                    "ocr_pattern": ocr_pattern,
                    "ocr_confidence": float(ocr_result.confidence or 0.0),
                    "ocr_prices": list(ocr_result.prices or []),
                }
            except Exception:
                ocr_payload = {"chart_bias": "unknown", "ocr_pattern": "", "ocr_confidence": 0.0}

        pattern = "range"
        confidence = 0.55
        if slope > 0.02 and volatility > 5.0:
            pattern = "uptrend"
            confidence = min(0.95, 0.6 + min(abs(slope) * 4.0, 0.2) + min(edge_density * 2.0, 0.15))
        elif slope < -0.02 and volatility > 5.0:
            pattern = "downtrend"
            confidence = min(0.95, 0.6 + min(abs(slope) * 4.0, 0.2) + min(edge_density * 2.0, 0.15))
        elif edge_density > 0.12 and volatility > 8.0:
            pattern = "breakout"
            confidence = min(0.95, 0.58 + min(edge_density * 1.5, 0.2))
        elif volatility < 4.0:
            pattern = "compression"
            confidence = 0.62
        if chart_bias == "bullish" and pattern in {"uptrend", "breakout"}:
            confidence = min(0.98, confidence + 0.07)
        elif chart_bias == "bearish" and pattern in {"downtrend", "breakout"}:
            confidence = min(0.98, confidence + 0.07)
        elif chart_bias in {"bullish", "bearish"} and pattern == "range":
            pattern = "bias_range"
            confidence = min(0.85, confidence + 0.04)

        return PatternResult(
            pattern=pattern,
            confidence=round(confidence, 4),
            payload={
                "frame_shape": [int(h), int(w)],
                "edge_density": round(edge_density, 6),
                "slope": round(slope, 6),
                "volatility": round(volatility, 4),
                **ocr_payload,
            },
        )
