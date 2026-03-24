"""insta360_capture.py — Captura frames multi-modo para vision de trading.

Prioridad de fuente (configurable via env vars):
  1. RTMP stream  — env INSTA360_RTMP_URL     (ej: rtmp://localhost:1935/live)
  2. Camera USB   — env INSTA360_CAMERA_INDEX (default 0, la Insta360 aparece como camara USB)
  3. Desktop PIL  — captura toda la pantalla (siempre disponible si Pillow instalado)

La clase no mantiene la camara abierta entre llamadas para evitar conflictos con otras apps.
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

logger = logging.getLogger("atlas.vision.insta360")


@dataclass
class CaptureResult:
    ok: bool
    frame: Optional[np.ndarray] = None
    source: str = ""
    latency_ms: float = 0.0
    error: Optional[str] = None


class InstaCapture:
    """Captura un frame desde la Insta360 o fallback a desktop screenshot.

    Uso::

        cam = InstaCapture()
        r = cam.capture()
        if r.ok:
            frame = r.frame   # np.ndarray BGR, listo para ChartOCR
    """

    def __init__(
        self,
        rtmp_url: Optional[str] = None,
        camera_index: Optional[int] = None,
        prefer_desktop: bool = False,
    ) -> None:
        self.rtmp_url = rtmp_url or os.getenv("INSTA360_RTMP_URL", "")
        if camera_index is not None:
            self.camera_index = camera_index
        else:
            try:
                self.camera_index = int(os.getenv("INSTA360_CAMERA_INDEX", "0"))
            except (ValueError, TypeError):
                self.camera_index = 0
        self.prefer_desktop = prefer_desktop

    # ── Captura principal ──────────────────────────────────────────────────────

    def capture(self, timeout_sec: float = 5.0) -> CaptureResult:
        t0 = time.perf_counter()

        if self.prefer_desktop:
            result = self._desktop()
        elif self.rtmp_url:
            result = self._rtmp(self.rtmp_url, timeout_sec)
            if not result.ok:
                logger.debug("RTMP fallo (%s), intentando camara local", result.error)
                result = self._cv2(self.camera_index, timeout_sec)
            if not result.ok:
                result = self._desktop()
        else:
            result = self._cv2(self.camera_index, timeout_sec)
            if not result.ok:
                result = self._desktop()

        result.latency_ms = round((time.perf_counter() - t0) * 1000, 1)
        if result.ok:
            logger.debug("Captura OK source=%s latency=%.0fms shape=%s",
                         result.source, result.latency_ms,
                         result.frame.shape if result.frame is not None else "?")
        else:
            logger.warning("Captura fallida todos los modos: %s", result.error)
        return result

    def source_available(self) -> str:
        """Primer modo disponible: 'rtmp' | 'camera' | 'desktop' | 'none'."""
        if self.rtmp_url:
            if self._rtmp(self.rtmp_url, timeout_sec=3.0).ok:
                return "rtmp"
        if self._cv2(self.camera_index, timeout_sec=3.0).ok:
            return "camera"
        if self._desktop().ok:
            return "desktop"
        return "none"

    # ── Backends ──────────────────────────────────────────────────────────────

    def _rtmp(self, url: str, timeout_sec: float) -> CaptureResult:
        try:
            import cv2  # type: ignore
            cap = cv2.VideoCapture(url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            deadline = time.time() + timeout_sec
            ret, frame = False, None
            while time.time() < deadline:
                ret, frame = cap.read()
                if ret and frame is not None:
                    break
                time.sleep(0.1)
            cap.release()
            if ret and frame is not None:
                return CaptureResult(ok=True, frame=frame, source=f"rtmp:{url[:50]}")
            return CaptureResult(ok=False, source="rtmp", error="timeout sin frame")
        except ImportError:
            return CaptureResult(ok=False, source="rtmp", error="cv2 no instalado")
        except Exception as exc:
            return CaptureResult(ok=False, source="rtmp", error=str(exc))

    def _cv2(self, index: int, timeout_sec: float) -> CaptureResult:
        try:
            import cv2  # type: ignore
            # CAP_DSHOW reduce latencia en Windows
            backend = getattr(cv2, "CAP_DSHOW", 0)
            cap = cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                cap.release()
                return CaptureResult(ok=False, source=f"cv2:{index}",
                                     error="camara no disponible")
            # Vaciar buffer (descartar frames viejos)
            for _ in range(3):
                cap.read()
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                return CaptureResult(ok=True, frame=frame, source=f"camera:index{index}")
            return CaptureResult(ok=False, source=f"cv2:{index}", error="frame vacio")
        except ImportError:
            return CaptureResult(ok=False, source=f"cv2:{index}", error="cv2 no instalado")
        except Exception as exc:
            return CaptureResult(ok=False, source=f"cv2:{index}", error=str(exc))

    @staticmethod
    def _desktop() -> CaptureResult:
        try:
            from PIL import ImageGrab  # type: ignore
            img = ImageGrab.grab(all_screens=True)
            # PIL RGB → numpy BGR para consistencia con OpenCV
            frame = np.array(img.convert("RGB"))[:, :, ::-1].copy()
            return CaptureResult(ok=True, frame=frame, source="desktop")
        except ImportError:
            return CaptureResult(ok=False, source="desktop",
                                 error="Pillow no instalado (pip install Pillow)")
        except Exception as exc:
            return CaptureResult(ok=False, source="desktop", error=str(exc))
