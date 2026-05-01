"""insta360_capture.py — Captura frames multi-modo para vision de trading.

Prioridad de fuente (configurable via env vars):
  1. RTMP stream  — env INSTA360_RTMP_URL     (ej: rtmp://localhost:1935/live)
  2. Camera USB   — env INSTA360_CAMERA_INDEX (solo ese indice si la variable **esta definida**)
  3. En Windows, si INSTA360_CAMERA_INDEX no esta definido: barrido acotado 0..INSTA360_USB_SCAN_MAX (def. 3)
  4. Desktop PIL  — captura de pantalla (fallback operativo)

Backend USB en Windows (orden fijo, documentado):
  - **DirectShow (CAP_DSHOW) primero**: menor latencia y mismo criterio que el probe historico del repo
    (``scripts/windows/atlas_robot_camera_bridge.ps1``); muchas webcams/Insta360 en modo UVC responden bien.
  - **Media Foundation (CAP_MSMF) segundo**: algunos drivers solo exponen bien el dispositivo por MSMF;
    evita quedar bloqueado si DSHOW abre pero no entrega frames.

La clase no mantiene la camara abierta entre llamadas para evitar conflictos con otras apps.
"""
from __future__ import annotations

import logging
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Iterator, Optional

import numpy as np

logger = logging.getLogger("atlas.vision.insta360")


@dataclass
class CaptureResult:
    ok: bool
    frame: Optional[np.ndarray] = None
    source: str = ""
    latency_ms: float = 0.0
    error: Optional[str] = None


def _iter_usb_backends(cv2: Any) -> Iterator[tuple[str, int]]:
    """Orden de backends para captura USB (Windows vs resto)."""
    if sys.platform == "win32":
        if hasattr(cv2, "CAP_DSHOW"):
            yield "dshow", int(cv2.CAP_DSHOW)
        if hasattr(cv2, "CAP_MSMF"):
            yield "msmf", int(cv2.CAP_MSMF)
        return
    yield "any", int(cv2.CAP_ANY)


def _usb_scan_max() -> int:
    try:
        return max(0, min(int(os.getenv("INSTA360_USB_SCAN_MAX", "3")), 8))
    except (ValueError, TypeError):
        return 3


def _explicit_usb_index_from_env() -> Optional[int]:
    """Si INSTA360_CAMERA_INDEX esta definido en el entorno, solo se usa ese indice (USB)."""
    if "INSTA360_CAMERA_INDEX" not in os.environ:
        return None
    raw = os.environ.get("INSTA360_CAMERA_INDEX", "0").strip()
    try:
        return int(raw)
    except (ValueError, TypeError):
        return 0


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
        self._last_usb_backend: str | None = None
        self._last_resolved_usb_index: int | None = None

    # ── Captura principal ──────────────────────────────────────────────────────

    def capture(self, timeout_sec: float = 5.0) -> CaptureResult:
        t0 = time.perf_counter()

        if self.prefer_desktop:
            result = self._desktop()
        elif self.rtmp_url:
            result = self._rtmp(self.rtmp_url, timeout_sec)
            if not result.ok:
                logger.debug("RTMP fallo (%s), intentando camara local", result.error)
                result = self._capture_usb_with_priority(timeout_sec)
            if not result.ok:
                result = self._desktop()
        else:
            result = self._capture_usb_with_priority(timeout_sec)
            if not result.ok:
                result = self._desktop()

        result.latency_ms = round((time.perf_counter() - t0) * 1000, 1)
        if result.ok:
            logger.debug(
                "Captura OK source=%s latency=%.0fms shape=%s",
                result.source,
                result.latency_ms,
                result.frame.shape if result.frame is not None else "?",
            )
        else:
            logger.warning("Captura fallida todos los modos: %s", result.error)
        return result

    def _capture_usb_with_priority(self, timeout_sec: float) -> CaptureResult:
        """RTMP ya descartado o vacio: USB segun prioridad indice explicito / barrido Windows."""
        explicit = _explicit_usb_index_from_env()
        if explicit is not None:
            return self._cv2_at_index(explicit, timeout_sec)
        if sys.platform == "win32":
            for idx in range(0, _usb_scan_max() + 1):
                r = self._cv2_at_index(idx, min(timeout_sec, 2.0))
                if r.ok:
                    return r
            return CaptureResult(ok=False, source="cv2:scan", error="ningun indice USB en rango")
        return self._cv2_at_index(self.camera_index, timeout_sec)

    def source_available(self) -> str:
        """Primer modo disponible: 'rtmp' | 'camera' | 'desktop' | 'none'."""
        self._last_usb_backend = None
        self._last_resolved_usb_index = None
        if self.rtmp_url:
            if self._rtmp(self.rtmp_url, timeout_sec=3.0).ok:
                return "rtmp"
        explicit = _explicit_usb_index_from_env()
        if explicit is not None:
            if self._cv2_at_index(explicit, timeout_sec=3.0).ok:
                self._last_resolved_usb_index = explicit
                return "camera"
        elif sys.platform == "win32":
            for idx in range(0, _usb_scan_max() + 1):
                if self._cv2_at_index(idx, timeout_sec=2.0).ok:
                    self._last_resolved_usb_index = idx
                    return "camera"
        else:
            if self._cv2_at_index(self.camera_index, timeout_sec=3.0).ok:
                self._last_resolved_usb_index = self.camera_index
                return "camera"
        if self._desktop().ok:
            return "desktop"
        return "none"

    def build_health_snapshot(self, *, pnp_timeout_sec: float = 2.5) -> dict[str, Any]:
        """Diagnostico estructurado para /camera/health (Quant); no persiste estado."""
        hints: list[dict[str, Any]] = []
        hints_short: list[str] = []
        if sys.platform == "win32":
            try:
                try:
                    from atlas_code_quant.vision.windows_camera_hints import fetch_pnp_camera_hints
                except ImportError:
                    from vision.windows_camera_hints import fetch_pnp_camera_hints  # type: ignore

                hints = fetch_pnp_camera_hints(timeout_sec=pnp_timeout_sec)
                hints_short = [
                    f"{h.get('class') or 'Camera'}:{(h.get('name') or '')[:100]}"
                    for h in hints[:10]
                ]
            except Exception as exc:
                logger.debug("build_health_snapshot PnP skip: %s", exc)

        mode = self.source_available()
        backend: str | None = None
        if mode == "rtmp":
            backend = "rtmp"
        elif mode == "camera":
            backend = self._last_usb_backend
        elif mode == "desktop":
            backend = "desktop_pil"

        notes_parts = []
        if self.rtmp_url:
            notes_parts.append("INSTA360_RTMP_URL definido")
        if "INSTA360_CAMERA_INDEX" in os.environ:
            notes_parts.append(f"INSTA360_CAMERA_INDEX={os.environ.get('INSTA360_CAMERA_INDEX')}")
        elif sys.platform == "win32":
            notes_parts.append(f"barrido USB 0..{_usb_scan_max()} (DSHOW luego MSMF)")
        if self._last_resolved_usb_index is not None:
            notes_parts.append(f"indice_resuelto={self._last_resolved_usb_index}")
        if hints_short:
            notes_parts.append("PnP:" + "; ".join(hints_short[:4]))

        return {
            "mode_detected": mode,
            "backend": backend,
            "device_index": self._last_resolved_usb_index,
            "pnp_hints": hints_short,
            "probe_notes": " ".join(notes_parts) if notes_parts else "",
        }

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

    def _cv2_at_index(self, index: int, timeout_sec: float) -> CaptureResult:
        try:
            import cv2  # type: ignore
        except ImportError:
            return CaptureResult(ok=False, source=f"cv2:{index}", error="cv2 no instalado")

        self._last_usb_backend = None
        last_err: str | None = None
        for bname, backend in _iter_usb_backends(cv2):
            cap = None
            try:
                cap = cv2.VideoCapture(index, backend)
                if not cap.isOpened():
                    last_err = f"{bname}:not_opened"
                    continue
                for _ in range(3):
                    cap.read()
                ret, frame = cap.read()
                if ret and frame is not None:
                    self._last_usb_backend = bname
                    return CaptureResult(ok=True, frame=frame, source=f"camera:{bname}:{index}")
                last_err = f"{bname}:frame_vacio"
            except Exception as exc:
                last_err = f"{bname}:{exc}"
            finally:
                if cap is not None:
                    cap.release()
        return CaptureResult(
            ok=False,
            source=f"cv2:{index}",
            error=last_err or "camara no disponible",
        )

    @staticmethod
    def _desktop() -> CaptureResult:
        try:
            from PIL import ImageGrab  # type: ignore

            img = ImageGrab.grab(all_screens=True)
            frame = np.array(img.convert("RGB"))[:, :, ::-1].copy()
            return CaptureResult(ok=True, frame=frame, source="desktop")
        except ImportError:
            return CaptureResult(ok=False, source="desktop", error="Pillow no instalado (pip install Pillow)")
        except Exception as exc:
            return CaptureResult(ok=False, source="desktop", error=str(exc))
