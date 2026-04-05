from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np


@dataclass(slots=True)
class GeometryHit:
    pattern: str
    classification: str
    confidence: float
    reason: str
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class GeometryDetectionResult:
    confidence: float
    hits: list[GeometryHit] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)


class GeometryDetector:
    """Heuristic geometric detector with short temporal consistency memory."""

    def __init__(self, history_size: int = 12) -> None:
        self._history: deque[str] = deque(maxlen=max(4, history_size))

    def detect(self, processed_frame: np.ndarray | None) -> GeometryDetectionResult:
        if processed_frame is None or getattr(processed_frame, "size", 0) == 0:
            return GeometryDetectionResult(confidence=0.0, payload={"reason": "empty_frame"})

        frame = processed_frame
        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(frame, 60, 180)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60, minLineLength=50, maxLineGap=10)
        line_metrics = self._line_metrics(lines)
        trend_metrics = self._trend_metrics(frame)

        hits: list[GeometryHit] = []
        triangle = self._detect_triangle(line_metrics, trend_metrics)
        if triangle is not None:
            hits.append(triangle)
        channel = self._detect_channel(line_metrics, trend_metrics)
        if channel is not None:
            hits.append(channel)
        hns = self._detect_head_and_shoulders(trend_metrics)
        if hns is not None:
            hits.append(hns)

        if hits:
            self._history.append(hits[0].pattern)
        consistency = self._consistency_score(hits[0].pattern if hits else "")
        for hit in hits:
            hit.confidence = round(min(1.0, hit.confidence + consistency * 0.15), 4)

        hits.sort(key=lambda item: item.confidence, reverse=True)
        return GeometryDetectionResult(
            confidence=hits[0].confidence if hits else 0.0,
            hits=hits,
            payload={
                "line_count": line_metrics["count"],
                "dominant_slope": round(line_metrics["dominant_slope"], 4),
                "slope_std": round(line_metrics["slope_std"], 4),
                "consistency_score": round(consistency, 4),
            },
        )

    @staticmethod
    def _line_metrics(lines: Any) -> dict[str, float]:
        if lines is None:
            return {"count": 0, "dominant_slope": 0.0, "slope_std": 0.0, "parallel_ratio": 0.0}
        slopes: list[float] = []
        for line in lines[:, 0]:
            x1, y1, x2, y2 = [float(v) for v in line]
            dx = x2 - x1
            if abs(dx) < 1e-6:
                continue
            slopes.append((y2 - y1) / dx)
        if not slopes:
            return {"count": 0, "dominant_slope": 0.0, "slope_std": 0.0, "parallel_ratio": 0.0}
        arr = np.asarray(slopes, dtype=float)
        dominant = float(np.median(arr))
        std = float(np.std(arr))
        parallel_ratio = float(np.mean(np.abs(arr - dominant) < 0.15))
        return {
            "count": float(len(arr)),
            "dominant_slope": dominant,
            "slope_std": std,
            "parallel_ratio": parallel_ratio,
        }

    @staticmethod
    def _trend_metrics(frame: np.ndarray) -> dict[str, float]:
        profile = frame.mean(axis=0)
        x = np.arange(len(profile), dtype=float)
        if len(profile) < 6:
            return {"slope": 0.0, "volatility": 0.0, "peak_ratio": 0.0}
        slope = float(np.polyfit(x, profile, 1)[0])
        volatility = float(np.std(profile))
        smooth = np.convolve(profile, np.ones(7) / 7.0, mode="same")
        peaks = int(np.sum((smooth[1:-1] > smooth[:-2]) & (smooth[1:-1] > smooth[2:])))
        troughs = int(np.sum((smooth[1:-1] < smooth[:-2]) & (smooth[1:-1] < smooth[2:])))
        peak_ratio = float(min(peaks, troughs) / max(max(peaks, troughs), 1))
        return {"slope": slope, "volatility": volatility, "peak_ratio": peak_ratio}

    def _detect_triangle(self, line_metrics: dict[str, float], trend_metrics: dict[str, float]) -> GeometryHit | None:
        if line_metrics["count"] < 4 or trend_metrics["volatility"] < 4.0:
            return None
        converging = line_metrics["slope_std"] > 0.25 and abs(line_metrics["dominant_slope"]) < 0.4
        if not converging:
            return None
        confidence = 0.58 + min(line_metrics["slope_std"] * 0.18, 0.2)
        return GeometryHit(
            pattern="triangle",
            classification="fuerte" if confidence >= 0.75 else "debil",
            confidence=confidence,
            reason="hough_convergence",
            metrics={**line_metrics, **trend_metrics},
        )

    def _detect_channel(self, line_metrics: dict[str, float], trend_metrics: dict[str, float]) -> GeometryHit | None:
        if line_metrics["count"] < 4 or line_metrics["parallel_ratio"] < 0.45:
            return None
        confidence = 0.55 + min(line_metrics["parallel_ratio"] * 0.25, 0.2)
        classification = "fuerte" if confidence >= 0.75 else "debil"
        if trend_metrics["volatility"] < 3.0:
            classification = "ambiguo"
            confidence = min(confidence, 0.68)
        return GeometryHit(
            pattern="channel",
            classification=classification,
            confidence=confidence,
            reason="parallel_trendlines",
            metrics={**line_metrics, **trend_metrics},
        )

    def _detect_head_and_shoulders(self, trend_metrics: dict[str, float]) -> GeometryHit | None:
        if trend_metrics["peak_ratio"] < 0.55 or trend_metrics["volatility"] < 6.0:
            return None
        confidence = 0.5 + min(trend_metrics["peak_ratio"] * 0.22, 0.22)
        classification = "probable_falso_positivo" if abs(trend_metrics["slope"]) > 0.8 else "ambiguo"
        if confidence >= 0.72 and abs(trend_metrics["slope"]) <= 0.45:
            classification = "debil"
        return GeometryHit(
            pattern="head_and_shoulders",
            classification=classification,
            confidence=confidence,
            reason="peak_symmetry",
            metrics=trend_metrics,
        )

    def _consistency_score(self, pattern: str) -> float:
        if not pattern:
            return 0.0
        if not self._history:
            return 0.0
        matches = sum(1 for item in self._history if item == pattern)
        return matches / float(len(self._history))
