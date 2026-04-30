"""Módulo 1A — Insta360 X4/X5 Camera Interface + OCR Pipeline.

Hardware target: NVIDIA Jetson Orin Nano (ARM64, CUDA 11.4+)
Stream: RTMP 2880×1440 @ 30fps → frame queue → YOLOv8 → EasyOCR / Qwen2-VL

Flujo:
    CameraInterface.start()
        └─ _capture_thread()    ← lee RTMP, pone frames en queue
        └─ _process_thread()    ← corre YOLO → OCR por ROI → publica OCRResult
"""

from __future__ import annotations

import logging
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

logger = logging.getLogger("atlas.hardware.camera")

# ── Intentar importar dependencias opcionales del Jetson ──────────────────────
try:
    import easyocr
    _EASYOCR_OK = True
except ImportError:
    _EASYOCR_OK = False
    logger.warning("easyocr no disponible — OCR deshabilitado")

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False
    logger.warning("ultralytics no disponible — detección de pantallas deshabilitada")

try:
    from transformers import AutoProcessor, AutoModelForVision2Seq
    import torch
    _QWEN_OK = True
except ImportError:
    _QWEN_OK = False


# ── Dataclasses de resultado ──────────────────────────────────────────────────

@dataclass
class FrameCapture:
    """Frame capturado de Insta360 con metadatos."""
    frame: np.ndarray
    timestamp: float
    frame_id: int
    width: int
    height: int
    latency_ms: float = 0.0


@dataclass
class ScreenROI:
    """Región de pantalla detectada por YOLOv8."""
    bbox: tuple[int, int, int, int]   # x1, y1, x2, y2
    confidence: float
    label: str = "screen"


@dataclass
class OCRResult:
    """Resultado del pipeline OCR sobre una captura."""
    timestamp: float
    prices: list[float] = field(default_factory=list)
    raw_texts: list[str] = field(default_factory=list)
    greeks: dict[str, float] = field(default_factory=dict)
    chart_color: str = "unknown"        # bullish / bearish / neutral
    pattern_detected: str = ""
    screens_detected: int = 0
    brightness: float = 0.0
    sharpness: float = 0.0
    safe_mode: bool = False             # pantalla oscura → pausa trading
    capture_ms: float = 0.0
    ocr_confidence: float = 0.0


# ── Utilidades de imagen ──────────────────────────────────────────────────────

def _brightness(frame: np.ndarray) -> float:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(gray.mean())


def _sharpness(frame: np.ndarray) -> float:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _dominant_color(roi: np.ndarray) -> str:
    """Determina tendencia visual: más verde=bullish, más rojo=bearish."""
    b = float(roi[:, :, 0].mean())
    g = float(roi[:, :, 1].mean())
    r = float(roi[:, :, 2].mean())
    if g > r * 1.15:
        return "bullish"
    if r > g * 1.15:
        return "bearish"
    return "neutral"


def _parse_prices_from_text(texts: list[str]) -> list[float]:
    """Extrae números flotantes (precios) desde textos OCR."""
    import re
    prices: list[float] = []
    pattern = re.compile(r"\$?(\d{1,6}[.,]\d{1,4})")
    for t in texts:
        for m in pattern.findall(t):
            try:
                prices.append(float(m.replace(",", ".")))
            except ValueError:
                pass
    return sorted(set(prices))


def _parse_greeks(texts: list[str]) -> dict[str, float]:
    """Extrae Greeks (delta, gamma, theta, vega) de textos OCR."""
    import re
    greeks: dict[str, float] = {}
    labels = {"delta": r"[Δd]elta\s*:?\s*(-?\d+\.\d+)",
              "gamma": r"[Gg]amma\s*:?\s*(\d+\.\d+)",
              "theta": r"[Θt]heta\s*:?\s*(-?\d+\.\d+)",
              "vega":  r"[Vv]ega\s*:?\s*(\d+\.\d+)",
              "iv":    r"IV\s*:?\s*(\d+\.?\d*)%?"}
    combined = " ".join(texts)
    for name, pat in labels.items():
        m = re.search(pat, combined)
        if m:
            try:
                greeks[name] = float(m.group(1))
            except ValueError:
                pass
    return greeks


# ── Cámara Principal ──────────────────────────────────────────────────────────

class CameraInterface:
    """Interface completa para Insta360 X4/X5 vía RTMP.

    Ejemplo de uso::

        cam = CameraInterface(rtmp_url="rtmp://192.168.1.10/live/atlas")
        cam.on_result(lambda r: print(r.prices))
        cam.start()
        # ... trading loop ...
        cam.stop()
    """

    # Calidad OCR mínima para operar — si cae por debajo → self-healing
    MIN_OCR_CONFIDENCE = 0.92
    # Latencia máxima de captura antes de activar modo físico (ms)
    MAX_LATENCY_MS = 200.0
    # Intervalo de procesamiento (s) para no saturar Jetson
    PROCESS_INTERVAL_S = 0.1

    def __init__(
        self,
        rtmp_url: "str | int" = "rtmp://192.168.1.10/live/atlas",
        yolo_model_path: str = "yolov8n.pt",
        ocr_languages: list[str] | None = None,
        frame_queue_size: int = 30,
        use_gpu: bool = True,
        qwen_model: str | None = None,
    ) -> None:
        self.rtmp_url = rtmp_url
        self.yolo_model_path = yolo_model_path
        self.ocr_languages = ocr_languages or ["en", "es"]
        self.frame_queue_size = frame_queue_size
        self.use_gpu = use_gpu
        self.qwen_model = qwen_model

        self._frame_queue: queue.Queue[FrameCapture] = queue.Queue(maxsize=frame_queue_size)
        self._result_callbacks: list[Callable[[OCRResult], None]] = []
        self._latest_result: Optional[OCRResult] = None
        self._running = False
        self._frame_id = 0
        self._cap: Optional[cv2.VideoCapture] = None

        # Modelos (lazy init en start())
        self._yolo: object | None = None
        self._ocr_reader: object | None = None
        self._qwen_processor: object | None = None
        self._qwen_model_obj: object | None = None

        # Métricas de calidad
        self._ocr_confidence_history: list[float] = []
        self._latency_history: list[float] = []
        self._on_degraded: list[Callable[[str, float], None]] = []

        self._capture_thread: Optional[threading.Thread] = None
        self._process_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    # ── Configuración de callbacks ────────────────────────────────────────────

    def on_result(self, callback: Callable[[OCRResult], None]) -> None:
        """Registra callback invocado con cada OCRResult procesado."""
        self._result_callbacks.append(callback)

    def on_degraded(self, callback: Callable[[str, float], None]) -> None:
        """Callback cuando OCR <92% o latencia >200ms. Params: (motivo, valor)."""
        self._on_degraded.append(callback)

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicializa modelos y arranca threads de captura y procesamiento."""
        self._init_models()
        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="atlas-cam-capture"
        )
        self._process_thread = threading.Thread(
            target=self._process_loop, daemon=True, name="atlas-cam-ocr"
        )
        self._capture_thread.start()
        self._process_thread.start()
        logger.info("CameraInterface iniciada → %s", self.rtmp_url)

    def stop(self) -> None:
        self._running = False
        if self._cap:
            self._cap.release()
        logger.info("CameraInterface detenida")

    def latest_result(self) -> Optional[OCRResult]:
        with self._lock:
            return self._latest_result

    # ── Inicialización de modelos ─────────────────────────────────────────────

    def _init_models(self) -> None:
        """Carga YOLOv8, EasyOCR y opcionalmente Qwen2-VL en GPU/CPU."""
        cuda_available = False
        if self.use_gpu:
            try:
                import torch
                cuda_available = bool(torch.cuda.is_available())
            except Exception:
                cuda_available = False
        if self.use_gpu and not cuda_available:
            logger.info("use_gpu=True pero CUDA no disponible — usando CPU para visión")
        yolo_device = "cuda" if (self.use_gpu and _YOLO_OK and cuda_available) else "cpu"
        ocr_gpu = bool(self.use_gpu and _EASYOCR_OK and cuda_available)

        if _YOLO_OK:
            self._yolo = _YOLO(self.yolo_model_path)
            self._yolo.to(yolo_device)  # type: ignore[union-attr]
            logger.info("YOLOv8 cargado en %s", yolo_device)

        if _EASYOCR_OK:
            self._ocr_reader = easyocr.Reader(self.ocr_languages, gpu=ocr_gpu)  # type: ignore[attr-defined]
            logger.info("EasyOCR listo (gpu=%s)", ocr_gpu)

        if _QWEN_OK and self.qwen_model:
            try:
                import torch
                qwen_gpu = bool(self.use_gpu and cuda_available)
                self._qwen_processor = AutoProcessor.from_pretrained(self.qwen_model)
                self._qwen_model_obj = AutoModelForVision2Seq.from_pretrained(
                    self.qwen_model,
                    torch_dtype=torch.float16 if qwen_gpu else torch.float32,
                    device_map="auto" if qwen_gpu else "cpu",
                )
                logger.info("Qwen2-VL cargado: %s", self.qwen_model)
            except Exception as exc:
                logger.warning("Qwen2-VL falló al cargar: %s", exc)

    # ── Thread de captura RTMP ────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        """Abre stream RTMP o cámara USB y pone frames en la queue."""
        retry_delay = 2.0
        # Soporte índice entero (USB/DirectShow) además de URL RTMP
        _src = self.rtmp_url
        if isinstance(_src, str) and _src.lstrip("-").isdigit():
            _src = int(_src)
        # USB entero: misma prioridad DSHOW→MSMF que InstaCapture (Windows estable).
        def _open_usb_int(idx: int):
            if hasattr(cv2, "CAP_DSHOW"):
                c = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                if c.isOpened():
                    return c
                c.release()
            if hasattr(cv2, "CAP_MSMF"):
                c = cv2.VideoCapture(idx, cv2.CAP_MSMF)
                if c.isOpened():
                    return c
                c.release()
            return cv2.VideoCapture(idx)

        while self._running:
            if isinstance(_src, int):
                self._cap = _open_usb_int(_src)
            else:
                self._cap = cv2.VideoCapture(_src)
            if not self._cap.isOpened():
                logger.error("No se pudo abrir fuente: %s — reintentando en %.0fs",
                             self.rtmp_url, retry_delay)
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 1.5, 30.0)
                continue

            retry_delay = 2.0
            logger.info("Stream RTMP abierto: %s", self.rtmp_url)

            while self._running:
                t0 = time.monotonic()
                ret, frame = self._cap.read()
                if not ret:
                    logger.warning("Stream perdido — reconectando…")
                    break

                latency_ms = (time.monotonic() - t0) * 1000
                h, w = frame.shape[:2]
                fc = FrameCapture(
                    frame=frame,
                    timestamp=time.time(),
                    frame_id=self._frame_id,
                    width=w,
                    height=h,
                    latency_ms=latency_ms,
                )
                self._frame_id += 1

                try:
                    self._frame_queue.put_nowait(fc)
                except queue.Full:
                    # Descarta frame antiguo — priorizamos latencia baja
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self._frame_queue.put_nowait(fc)

                # Control de velocidad: ~30 fps max
                elapsed = time.monotonic() - t0
                sleep_s = max(0.0, (1.0 / 30.0) - elapsed)
                if sleep_s > 0:
                    time.sleep(sleep_s)

            if self._cap:
                self._cap.release()

    # ── Thread de procesamiento OCR ───────────────────────────────────────────

    def _process_loop(self) -> None:
        """Toma frames de la queue, corre YOLO+OCR, publica OCRResult."""
        while self._running:
            try:
                fc: FrameCapture = self._frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            t0 = time.monotonic()
            result = self._process_frame(fc)
            result.capture_ms = (time.monotonic() - t0) * 1000

            # Guardar último resultado
            with self._lock:
                self._latest_result = result

            # Métricas de calidad
            self._latency_history.append(fc.latency_ms)
            if len(self._latency_history) > 100:
                self._latency_history.pop(0)

            # Detectar degradación
            if fc.latency_ms > self.MAX_LATENCY_MS:
                for cb in self._on_degraded:
                    cb("latency_ms", fc.latency_ms)

            if result.ocr_confidence < self.MIN_OCR_CONFIDENCE and result.screens_detected > 0:
                for cb in self._on_degraded:
                    cb("ocr_confidence", result.ocr_confidence)

            # Notificar callbacks
            for cb in self._result_callbacks:
                try:
                    cb(result)
                except Exception as exc:
                    logger.error("Error en callback de OCR: %s", exc)

            time.sleep(self.PROCESS_INTERVAL_S)

    def _process_frame(self, fc: FrameCapture) -> OCRResult:
        """Pipeline completo: YOLOv8 → ROI → EasyOCR → parsing."""
        result = OCRResult(timestamp=fc.timestamp)
        result.brightness = _brightness(fc.frame)
        result.sharpness = _sharpness(fc.frame)

        # Safe mode: pantalla muy oscura → pausar trading
        result.safe_mode = result.brightness < 20.0

        # ── YOLOv8: detectar regiones de pantalla ─────────────────────────
        rois: list[ScreenROI] = []
        if self._yolo is not None:
            try:
                detections = self._yolo(fc.frame, verbose=False)  # type: ignore[operator]
                for det in detections:
                    boxes = det.boxes
                    if boxes is None:
                        continue
                    for box in boxes:
                        conf = float(box.conf[0])
                        if conf < 0.4:
                            continue
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        rois.append(ScreenROI(bbox=(x1, y1, x2, y2), confidence=conf))
                result.screens_detected = len(rois)
            except Exception as exc:
                logger.debug("YOLO error: %s", exc)

        # Si no hay detecciones YOLO, usar frame completo como ROI
        if not rois:
            h, w = fc.frame.shape[:2]
            rois = [ScreenROI(bbox=(0, 0, w, h), confidence=1.0)]
            result.screens_detected = 0

        # ── EasyOCR sobre cada ROI ────────────────────────────────────────
        all_texts: list[str] = []
        all_confs: list[float] = []

        for roi in rois:
            x1, y1, x2, y2 = roi.bbox
            crop = fc.frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            # Color dominante
            if result.chart_color == "unknown":
                result.chart_color = _dominant_color(crop)

            if self._ocr_reader is not None:
                try:
                    ocr_out = self._ocr_reader.readtext(crop)  # type: ignore[attr-defined]
                    for (_, text, conf) in ocr_out:
                        if text.strip():
                            all_texts.append(text.strip())
                            all_confs.append(conf)
                except Exception as exc:
                    logger.debug("OCR error en ROI: %s", exc)

        result.raw_texts = all_texts
        result.prices = _parse_prices_from_text(all_texts)
        result.greeks = _parse_greeks(all_texts)
        result.ocr_confidence = float(np.mean(all_confs)) if all_confs else 0.0

        # ── Qwen2-VL: análisis de patrón avanzado (si disponible) ────────
        if self._qwen_model_obj is not None and rois:
            try:
                result.pattern_detected = self._qwen_analyze(fc.frame, rois[0])
            except Exception as exc:
                logger.debug("Qwen2-VL error: %s", exc)

        return result

    def _qwen_analyze(self, frame: np.ndarray, roi: ScreenROI) -> str:
        """Usa Qwen2-VL para identificar patrón técnico en la pantalla."""
        from PIL import Image
        import torch

        x1, y1, x2, y2 = roi.bbox
        crop = frame[y1:y2, x1:x2]
        img = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))

        prompt = (
            "Analiza este gráfico financiero. Identifica el patrón técnico principal "
            "(ej: doble techo, cabeza y hombros, triángulo, canal, bandera). "
            "Responde solo el nombre del patrón en una palabra o frase corta."
        )
        inputs = self._qwen_processor(  # type: ignore[operator]
            text=prompt, images=img, return_tensors="pt"
        ).to(next(self._qwen_model_obj.parameters()).device)  # type: ignore[union-attr]

        with torch.no_grad():
            output = self._qwen_model_obj.generate(  # type: ignore[union-attr]
                **inputs, max_new_tokens=30, do_sample=False
            )
        decoded = self._qwen_processor.decode(  # type: ignore[union-attr]
            output[0], skip_special_tokens=True
        )
        return decoded.strip()

    # ── Acceso rápido a precios ───────────────────────────────────────────────

    def get_latest_prices(self) -> list[float]:
        r = self.latest_result()
        return r.prices if r else []

    def get_ocr_confidence(self) -> float:
        r = self.latest_result()
        return r.ocr_confidence if r else 0.0

    def avg_latency_ms(self) -> float:
        if not self._latency_history:
            return 0.0
        return float(np.mean(self._latency_history))

    def to_dict(self) -> dict:
        r = self.latest_result()
        if not r:
            return {}
        return {
            "chart_color": r.chart_color,
            "ocr_prices": r.prices,
            "pattern_detected": r.pattern_detected,
            "capture_ms": r.capture_ms,
            "safe_mode": r.safe_mode,
            "screens_detected": r.screens_detected,
            "brightness": round(r.brightness, 1),
            "sharpness": round(r.sharpness, 1),
            "ocr_confidence": round(r.ocr_confidence, 4),
            "greeks": r.greeks,
            "raw_texts": r.raw_texts[:10],
        }
