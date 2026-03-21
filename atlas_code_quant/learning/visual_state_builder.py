"""Atlas Code-Quant — Constructor de estado visual para el agente RL.

Captura frames desde la cámara del robot (o Insta360 via RTMP),
procesa con OpenCV + YOLOv8 + EasyOCR para extraer features visuales
que enriquecen el estado del entorno RL.

Grok/xAI criterio 2026:
  "El agente aprende a leer charts visualmente (patrones head&shoulders,
   flags, etc.) mejor que APIs estáticas."
  "Si pantalla oscura o baja calidad → modo seguro."
  "OCR prioritario: precios bid/ask, volumen, timestamps."

Pipeline:
  Camera/RTMP frame
    → YOLOv8: detectar monitores/pantallas y regiones de chart
    → OpenCV: pre-procesar (contrast, denoise, crop)
    → EasyOCR: extraer texto (precios, porcentajes, volumen)
    → Feature vector 20-dim (numérico, listo para RL obs)
    → Safety check: brillo/nitidez → modo seguro si degradado

Uso::
    builder = VisualStateBuilder(camera_index=0)
    state = builder.capture_and_build()
    if state.safe_mode:
        # No operar con datos visuales
    else:
        obs = np.concatenate([tech_features, state.feature_vector, portfolio_state])
"""
from __future__ import annotations

import logging
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger("quant.learning.visual_state")

# ── Constantes ────────────────────────────────────────────────────────────────
_MIN_BRIGHTNESS  = 40.0    # 0-255 — por debajo → modo seguro
_MIN_SHARPNESS   = 50.0    # Laplacian variance — por debajo → borroso
_FEATURE_DIM     = 20      # Dimensión del vector visual de salida
_ZERO_FEATURES   = np.zeros(_FEATURE_DIM, dtype=np.float32)
_INSTA360_RTMP   = "rtmp://192.168.42.1/live/preview"  # Default WiFi Insta360


@dataclass
class VisualState:
    """Estado visual extraído de un frame de la cámara."""
    safe_mode: bool = True              # True = no usar datos visuales
    brightness: float = 0.0
    sharpness: float = 0.0
    screens_detected: int = 0
    ocr_prices: list[float] = field(default_factory=list)
    ocr_raw: list[str] = field(default_factory=list)
    chart_color: str = "unknown"        # "green" | "red" | "mixed" | "unknown"
    pattern_detected: str = ""          # "uptrend" | "downtrend" | "breakout" | ""
    bid: float = 0.0
    ask: float = 0.0
    volume_text: str = ""
    feature_vector: np.ndarray = field(default_factory=lambda: _ZERO_FEATURES.copy())
    capture_ms: float = 0.0
    source: str = ""
    reason: str = ""                    # Razón del safe_mode si aplica

    def to_dict(self) -> dict:
        return {
            "safe_mode":         self.safe_mode,
            "brightness":        round(self.brightness, 2),
            "sharpness":         round(self.sharpness, 2),
            "screens_detected":  self.screens_detected,
            "ocr_prices":        self.ocr_prices[:5],
            "chart_color":       self.chart_color,
            "pattern_detected":  self.pattern_detected,
            "bid":               self.bid,
            "ask":               self.ask,
            "feature_dim":       len(self.feature_vector),
            "capture_ms":        round(self.capture_ms, 1),
            "source":            self.source,
            "reason":            self.reason,
        }


class VisualStateBuilder:
    """Construye features visuales para el agente RL desde la cámara del robot.

    Args:
        camera_index: Índice de la cámara OpenCV (0 = webcam principal).
        use_rtmp: Si True, usa stream RTMP de Insta360 en lugar de cámara local.
        rtmp_url: URL del stream RTMP (Insta360 WiFi default).
        use_yolo: Si True, usa YOLOv8 para detectar pantallas/monitores.
        use_ocr: Si True, usa EasyOCR para extraer texto de precios.
        resolution: Resolución de captura (width, height).
        alert_dispatcher: Instancia de AlertDispatcher para notificar obstrucciones.

    Example::
        builder = VisualStateBuilder(camera_index=0, use_ocr=True)
        state = builder.capture_and_build()
        print(state.ocr_prices, state.feature_vector[:5])
    """

    def __init__(
        self,
        camera_index: int = 0,
        use_rtmp: bool = False,
        rtmp_url: str = _INSTA360_RTMP,
        use_yolo: bool = False,
        use_ocr: bool = True,
        resolution: tuple[int, int] = (640, 480),
        alert_dispatcher=None,
    ) -> None:
        self.camera_index  = camera_index
        self.use_rtmp      = use_rtmp
        self.rtmp_url      = rtmp_url
        self.use_yolo      = use_yolo
        self.use_ocr       = use_ocr
        self.resolution    = resolution
        self._dispatcher   = alert_dispatcher
        self._cap          = None       # cv2.VideoCapture
        self._yolo_model   = None
        self._ocr_reader   = None
        self._safe_mode_notified = False
        self._last_safe_ts = 0.0

    # ── API Pública ───────────────────────────────────────────────────────────

    def capture_and_build(self) -> VisualState:
        """Captura un frame y construye el estado visual completo.

        Returns:
            VisualState con features extraídas y vector numpy para RL.
            Si hay error o calidad degradada, safe_mode=True y features en cero.
        """
        t0 = time.time()
        try:
            import cv2
        except ImportError:
            return VisualState(safe_mode=True, reason="opencv_not_installed",
                               feature_vector=_ZERO_FEATURES.copy())

        frame = self._capture_frame(cv2)
        if frame is None:
            return VisualState(safe_mode=True, reason="capture_failed",
                               feature_vector=_ZERO_FEATURES.copy(), capture_ms=(time.time()-t0)*1000)

        # ── Quality checks ────────────────────────────────────────────────────
        brightness = self._compute_brightness(frame)
        sharpness  = self._compute_sharpness(cv2, frame)

        if brightness < _MIN_BRIGHTNESS:
            self._notify_obstruction("low_brightness", brightness=brightness)
            return VisualState(safe_mode=True, reason="low_brightness",
                               brightness=brightness, sharpness=sharpness,
                               feature_vector=_ZERO_FEATURES.copy(),
                               capture_ms=(time.time()-t0)*1000, source=self._source_name())

        if sharpness < _MIN_SHARPNESS:
            self._notify_obstruction("blurry_frame")
            return VisualState(safe_mode=True, reason="blurry_frame",
                               brightness=brightness, sharpness=sharpness,
                               feature_vector=_ZERO_FEATURES.copy(),
                               capture_ms=(time.time()-t0)*1000, source=self._source_name())

        # ── Modo seguro desactivado: buenos datos ─────────────────────────────
        self._safe_mode_notified = False

        state = VisualState(
            safe_mode=False,
            brightness=brightness,
            sharpness=sharpness,
            source=self._source_name(),
        )

        # ── Detección de pantallas (YOLOv8) ───────────────────────────────────
        chart_region = frame
        if self.use_yolo:
            chart_region, n_screens = self._detect_screens(cv2, frame)
            state.screens_detected = n_screens
        else:
            state.screens_detected = 1

        # ── Pre-procesado del chart ───────────────────────────────────────────
        processed = self._preprocess(cv2, chart_region)

        # ── Color dominante (tendencia visual) ───────────────────────────────
        state.chart_color = self._detect_color(processed)

        # ── OCR de precios ───────────────────────────────────────────────────
        if self.use_ocr:
            prices, raw_texts = self._extract_prices_ocr(processed)
            state.ocr_prices = prices
            state.ocr_raw    = raw_texts[:10]
            if len(prices) >= 2:
                state.bid = min(prices[:2])
                state.ask = max(prices[:2])

        # ── Patrón de chart (heurística visual simple) ────────────────────────
        state.pattern_detected = self._detect_pattern(processed, state.chart_color)

        # ── Construir vector de features ──────────────────────────────────────
        state.feature_vector = self._build_feature_vector(state, brightness, sharpness)
        state.capture_ms = (time.time() - t0) * 1000

        return state

    def release(self) -> None:
        """Libera la cámara."""
        if self._cap:
            try:
                import cv2
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def status(self) -> dict:
        return {
            "source":       self._source_name(),
            "use_yolo":     self.use_yolo,
            "use_ocr":      self.use_ocr,
            "cap_open":     bool(self._cap and self._cap.isOpened() if self._cap else False),
            "yolo_loaded":  self._yolo_model is not None,
            "ocr_loaded":   self._ocr_reader is not None,
        }

    # ── Captura ───────────────────────────────────────────────────────────────

    def _capture_frame(self, cv2) -> "np.ndarray | None":
        """Captura un frame desde cámara local o RTMP."""
        try:
            if self._cap is None or not self._cap.isOpened():
                source = self.rtmp_url if self.use_rtmp else self.camera_index
                self._cap = cv2.VideoCapture(source)
                if self.resolution:
                    self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.resolution[0])
                    self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

            ret, frame = self._cap.read()
            if not ret or frame is None:
                self._cap.release()
                self._cap = None
                return None
            return frame
        except Exception as exc:
            logger.warning("[VisualState] Captura fallida: %s", exc)
            return None

    # ── Métricas de calidad ───────────────────────────────────────────────────

    def _compute_brightness(self, frame: "np.ndarray") -> float:
        try:
            import cv2
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return float(gray.mean())
        except Exception:
            return 255.0

    def _compute_sharpness(self, cv2, frame: "np.ndarray") -> float:
        """Laplacian variance — más alto = más nítido."""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return float(cv2.Laplacian(gray, cv2.CV_64F).var())
        except Exception:
            return 999.0

    # ── YOLOv8 detección de pantallas ────────────────────────────────────────

    def _detect_screens(self, cv2, frame: "np.ndarray") -> tuple["np.ndarray", int]:
        """Detecta monitores/pantallas con YOLOv8. Retorna región del chart y n_screens."""
        if self._yolo_model is None:
            try:
                from ultralytics import YOLO
                self._yolo_model = YOLO("yolov8n.pt")  # nano — rápido en Jetson
                logger.info("[VisualState] YOLOv8 cargado")
            except Exception as exc:
                logger.warning("[VisualState] YOLOv8 no disponible: %s", exc)
                return frame, 1

        try:
            results = self._yolo_model(frame, verbose=False)
            # COCO classes: 62=TV, 63=laptop, 84=monitor
            screen_classes = {62, 63, 84}
            boxes = []
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls in screen_classes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        boxes.append((x1, y1, x2, y2))

            if boxes:
                # Usar la pantalla más grande como región principal
                x1, y1, x2, y2 = max(boxes, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                return frame[y1:y2, x1:x2], len(boxes)
        except Exception as exc:
            logger.debug("[VisualState] YOLO error: %s", exc)

        return frame, 1

    # ── Pre-procesado ─────────────────────────────────────────────────────────

    def _preprocess(self, cv2, frame: "np.ndarray") -> "np.ndarray":
        """Mejora contraste y reduce ruido para OCR."""
        try:
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            denoised = cv2.GaussianBlur(enhanced, (3, 3), 0)
            return denoised
        except Exception:
            return frame

    # ── Análisis de color ─────────────────────────────────────────────────────

    def _detect_color(self, frame: "np.ndarray") -> str:
        """Detecta si el chart es mayoritariamente verde o rojo (tendencia visual)."""
        try:
            import cv2
            # Convertir de gray a BGR si hace falta
            if len(frame.shape) == 2:
                bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                bgr = frame

            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            # Rango verde
            green_mask = cv2.inRange(hsv, (36, 50, 50), (89, 255, 255))
            # Rango rojo (dos rangos en HSV)
            red_mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
            red_mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
            red_mask  = cv2.bitwise_or(red_mask1, red_mask2)

            n_green = int(green_mask.sum() / 255)
            n_red   = int(red_mask.sum() / 255)
            total   = frame.shape[0] * frame.shape[1]

            if total == 0:
                return "unknown"
            green_pct = n_green / total
            red_pct   = n_red / total

            if green_pct > 0.05 and green_pct > red_pct * 1.5:
                return "green"
            elif red_pct > 0.05 and red_pct > green_pct * 1.5:
                return "red"
            elif green_pct > 0.03 or red_pct > 0.03:
                return "mixed"
            return "unknown"
        except Exception:
            return "unknown"

    # ── OCR ───────────────────────────────────────────────────────────────────

    def _extract_prices_ocr(self, frame: "np.ndarray") -> tuple[list[float], list[str]]:
        """Extrae precios numéricos del frame usando EasyOCR."""
        if self._ocr_reader is None:
            try:
                import easyocr
                self._ocr_reader = easyocr.Reader(["en"], verbose=False, gpu=False)
                logger.info("[VisualState] EasyOCR cargado")
            except ImportError:
                logger.debug("[VisualState] easyocr no instalado — pip install easyocr")
                return [], []
            except Exception as exc:
                logger.warning("[VisualState] EasyOCR init error: %s", exc)
                return [], []

        try:
            import cv2
            # EasyOCR necesita BGR o RGB
            if len(frame.shape) == 2:
                rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            else:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = self._ocr_reader.readtext(rgb, detail=1, paragraph=False)
            raw_texts = [r[1] for r in results if r[2] > 0.3]  # conf > 30%
            prices = self._parse_prices(raw_texts)
            return prices, raw_texts
        except Exception as exc:
            logger.debug("[VisualState] OCR error: %s", exc)
            return [], []

    def _parse_prices(self, texts: list[str]) -> list[float]:
        """Extrae valores numéricos de precio de textos OCR."""
        prices = []
        pattern = re.compile(r"\b(\d{1,8}[.,]\d{1,8})\b")
        for text in texts:
            clean = text.replace(",", "").replace(" ", "")
            for match in pattern.findall(clean):
                try:
                    val = float(match.replace(",", "."))
                    if 0.0001 < val < 10_000_000:
                        prices.append(val)
                except ValueError:
                    pass
        return sorted(set(prices))

    # ── Patrón de chart ───────────────────────────────────────────────────────

    def _detect_pattern(self, frame: "np.ndarray", color: str) -> str:
        """Heurística visual simple para detectar tendencia del chart."""
        try:
            import cv2
            h, w = frame.shape[:2]
            if h < 10 or w < 10:
                return ""
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # Dividir en mitad izquierda y derecha y comparar brillo promedio
            left  = float(gray[:, :w//2].mean())
            right = float(gray[:, w//2:].mean())
            diff  = right - left

            if color == "green" and diff > 5:
                return "uptrend"
            elif color == "red" and diff < -5:
                return "downtrend"
            elif abs(diff) < 3:
                return "sideways"
            return "breakout" if abs(diff) > 15 else ""
        except Exception:
            return ""

    # ── Feature vector ────────────────────────────────────────────────────────

    def _build_feature_vector(
        self, state: VisualState, brightness: float, sharpness: float
    ) -> np.ndarray:
        """Construye un vector de 20 features numéricas normalizadas para el RL.

        Índices:
          0: brightness normalizada (0-1)
          1: sharpness normalizada (0-1, cap 500)
          2: n_screens detectadas (0-1, cap 4)
          3: chart_color → green=1, red=-1, mixed=0, unknown=0
          4: pattern → uptrend=1, downtrend=-1, sideways=0, breakout=0.5
          5-9: top 5 precios OCR normalizados (log-scale, 0 si ausente)
          10: bid/ask spread normalizado
          11: n_precios_detectados (0-1, cap 10)
          12-19: reservado (0) — para CNN embeddings futuros
        """
        v = np.zeros(_FEATURE_DIM, dtype=np.float32)

        v[0] = min(brightness / 255.0, 1.0)
        v[1] = min(sharpness / 500.0, 1.0)
        v[2] = min(state.screens_detected / 4.0, 1.0)

        color_map = {"green": 1.0, "red": -1.0, "mixed": 0.0, "unknown": 0.0}
        v[3] = color_map.get(state.chart_color, 0.0)

        pattern_map = {"uptrend": 1.0, "downtrend": -1.0, "sideways": 0.0, "breakout": 0.5}
        v[4] = pattern_map.get(state.pattern_detected, 0.0)

        # Precios OCR normalizados en log-scale
        for i, price in enumerate(state.ocr_prices[:5]):
            if price > 0:
                v[5 + i] = math.log(price + 1) / 15.0  # normalizar ~0-1 para rangos comunes

        # Spread bid/ask
        if state.bid > 0 and state.ask > 0 and state.ask > state.bid:
            spread_pct = (state.ask - state.bid) / state.bid
            v[10] = min(spread_pct * 100, 1.0)

        v[11] = min(len(state.ocr_prices) / 10.0, 1.0)

        # Reemplazar NaN/Inf
        v = np.where(np.isfinite(v), v, 0.0).astype(np.float32)
        return v

    # ── Notificaciones ────────────────────────────────────────────────────────

    def _notify_obstruction(self, reason: str, **kwargs) -> None:
        """Notifica al AlertDispatcher de obstrucción (con cooldown)."""
        now = time.time()
        if now - self._last_safe_ts < 300 and self._safe_mode_notified:
            return  # No spam
        self._safe_mode_notified = True
        self._last_safe_ts = now
        if self._dispatcher:
            try:
                import asyncio
                coro = self._dispatcher.camera_obstruction(reason=reason, **kwargs)
                if asyncio.get_event_loop().is_running():
                    asyncio.create_task(coro)
            except Exception:
                pass

    def _source_name(self) -> str:
        return self.rtmp_url if self.use_rtmp else f"camera:{self.camera_index}"


# ── Singleton global ──────────────────────────────────────────────────────────
_builder: VisualStateBuilder | None = None


def get_visual_state_builder(
    camera_index: int = 0,
    use_rtmp: bool = False,
    use_ocr: bool = True,
    alert_dispatcher=None,
) -> VisualStateBuilder:
    """Retorna la instancia global del VisualStateBuilder (lazy singleton)."""
    global _builder
    if _builder is None:
        _builder = VisualStateBuilder(
            camera_index=camera_index,
            use_rtmp=use_rtmp,
            use_ocr=use_ocr,
            alert_dispatcher=alert_dispatcher,
        )
    return _builder
