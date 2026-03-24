"""visual_pipeline.py — Orquesta InstaCapture → ChartOCR → VisualOCRResult.

Singleton thread-safe con cache de 30 segundos para no saturar la camara.

Uso::

    from atlas_code_quant.vision.visual_pipeline import VisualPipeline

    pipeline = VisualPipeline.get_instance()
    result   = pipeline.analyze()          # VisualOCRResult (cacheado 30s)
    status   = pipeline.status()           # dict con info de disponibilidad
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from atlas_code_quant.vision.chart_ocr import ChartOCR, VisualOCRResult
from atlas_code_quant.vision.insta360_capture import InstaCapture

logger = logging.getLogger("atlas.vision.pipeline")


class VisualPipeline:
    """Singleton que mantiene ChartOCR + InstaCapture listos."""

    _instance: Optional["VisualPipeline"] = None
    _init_lock = threading.Lock()

    def __init__(
        self,
        prefer_desktop: bool = False,
        use_gpu: bool = False,
        cache_ttl_sec: float = 30.0,
    ) -> None:
        self._capture = InstaCapture(prefer_desktop=prefer_desktop)
        self._ocr = ChartOCR(use_gpu=use_gpu)
        self._cache_ttl = cache_ttl_sec
        self._last_result: Optional[VisualOCRResult] = None
        self._last_at: float = 0.0
        self._result_lock = threading.Lock()

    # ── Singleton ─────────────────────────────────────────────────────────────

    @classmethod
    def get_instance(cls, **kwargs) -> "VisualPipeline":
        if cls._instance is None:
            with cls._init_lock:
                if cls._instance is None:
                    cls._instance = cls(**kwargs)
        return cls._instance

    # ── API publica ────────────────────────────────────────────────────────────

    def analyze(self, max_age_sec: float = 30.0) -> VisualOCRResult:
        """Captura y analiza la pantalla/camara.

        Devuelve resultado cacheado si tiene menos de `max_age_sec` segundos.
        Nunca lanza excepciones — en caso de error devuelve VisualOCRResult con .error.
        """
        now = time.time()
        with self._result_lock:
            if self._last_result and (now - self._last_at) < max_age_sec:
                return self._last_result

        cap = self._capture.capture(timeout_sec=5.0)
        if not cap.ok or cap.frame is None:
            result = VisualOCRResult(
                error=f"Captura fallida ({cap.source}): {cap.error}",
                source=cap.source,
            )
            logger.warning("VisualPipeline capture failed: %s", result.error)
        else:
            result = self._ocr.analyze(cap.frame)
            result.source = cap.source
            logger.info(
                "VisualPipeline OK | source=%s color=%s prices=%s pattern=%s conf=%.2f %.0fms",
                cap.source, result.chart_color,
                result.prices[:3] if result.prices else [],
                result.pattern_detected,
                result.confidence, cap.latency_ms,
            )

        with self._result_lock:
            self._last_result = result
            self._last_at = now
        return result

    def status(self) -> dict:
        """Estado del pipeline para dashboards y endpoints REST."""
        source = self._capture.source_available()
        with self._result_lock:
            last = self._last_result
        return {
            "ocr_available": self._ocr._ocr_ok,
            "camera_source": source,
            "rtmp_url": self._capture.rtmp_url or None,
            "camera_index": self._capture.camera_index,
            "prefer_desktop": self._capture.prefer_desktop,
            "cache_ttl_sec": self._cache_ttl,
            "last_result": {
                "chart_color": last.chart_color,
                "prices_found": len(last.prices),
                "prices_sample": last.prices[:5],
                "pattern": last.pattern_detected,
                "confidence": last.confidence,
                "source": last.source,
                "error": last.error,
            } if last else None,
        }
