"""chart_ocr.py — Analiza frames de graficos de trading.

Extrae:
  - Precios visibles via EasyOCR (lazy import; funciona sin el si no esta instalado)
  - Color dominante del chart (bullish=verde / bearish=rojo) via OpenCV o numpy puro
  - Patron detectado (bandera, ruptura, pullback, etc.)

Produce VisualOCRResult compatible con VisualTriggerValidator.
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

logger = logging.getLogger("atlas.vision.chart_ocr")

_PRICE_RE = re.compile(r"\d{1,6}(?:[.,]\d{1,4})?")

_BULLISH_KW = {"breakout", "buy", "long", "bull", "support", "bounce", "up", "alcista"}
_BEARISH_KW = {"breakdown", "sell", "short", "bear", "resistance", "dump", "down", "bajista"}
_PATTERN_MAP = {
    "flag": "bandera", "pennant": "banderita", "wedge": "cuna",
    "triangle": "triangulo", "channel": "canal",
    "head": "hombro_cabeza", "double": "doble_techo",
    "inside": "inside_bar", "breakout": "ruptura", "pullback": "pullback",
}


@dataclass
class VisualOCRResult:
    """Resultado del analisis visual de un frame de grafico.

    Atributos compatibles con VisualTriggerValidator:
      .prices          — lista de precios numericos detectados
      .chart_color     — "bullish" | "bearish" | "neutral" | "unknown"
      .pattern_detected— nombre del patron o ""
    """
    prices: List[float] = field(default_factory=list)
    chart_color: str = "unknown"
    pattern_detected: str = ""
    raw_texts: List[str] = field(default_factory=list)
    confidence: float = 0.0
    frame_shape: tuple = ()
    source: str = ""
    error: Optional[str] = None


class ChartOCR:
    """Analiza un frame BGR/RGB de grafico de trading.

    Uso::

        ocr = ChartOCR()
        result = ocr.analyze(frame)   # frame es np.ndarray
    """

    def __init__(self, use_gpu: bool = False) -> None:
        self._reader = None
        self._ocr_ok = False
        self._use_gpu = use_gpu
        self._try_init_ocr()

    def _try_init_ocr(self) -> None:
        try:
            import easyocr  # type: ignore
            self._reader = easyocr.Reader(["en"], gpu=self._use_gpu, verbose=False)
            self._ocr_ok = True
            logger.info("EasyOCR cargado OK (gpu=%s)", self._use_gpu)
        except ImportError:
            logger.warning("EasyOCR no instalado — solo analisis de color activo. "
                           "Instalar con: pip install easyocr")
        except Exception as exc:
            logger.warning("EasyOCR no pudo inicializar: %s", exc)

    # ── API publica ────────────────────────────────────────────────────────────

    def analyze(self, frame: np.ndarray) -> VisualOCRResult:
        """Analiza el frame y devuelve VisualOCRResult."""
        result = VisualOCRResult(frame_shape=tuple(frame.shape[:2]))
        try:
            result.chart_color = self._detect_color(frame)

            if self._ocr_ok and self._reader is not None:
                texts = self._run_ocr(frame)
                result.raw_texts = texts
                result.prices = self._extract_prices(texts)
                result.pattern_detected = self._detect_pattern(texts, result.chart_color)
            else:
                result.pattern_detected = result.chart_color if result.chart_color != "unknown" else ""

            has_price = bool(result.prices)
            has_color = result.chart_color not in {"unknown"}
            result.confidence = round(
                (0.6 if has_price else 0.0) + (0.4 if has_color else 0.0), 3
            )
        except Exception as exc:
            result.error = str(exc)
            logger.warning("ChartOCR.analyze error: %s", exc)
        return result

    # ── Analisis de color ──────────────────────────────────────────────────────

    def _detect_color(self, frame: np.ndarray) -> str:
        """Detecta si el area reciente del grafico es bullish (verde) o bearish (rojo)."""
        try:
            import cv2  # type: ignore
            h, w = frame.shape[:2]
            roi = frame[:, w // 2:, :]  # mitad derecha = velas recientes
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # Verde: H 35-85
            green = cv2.countNonZero(cv2.inRange(hsv, (35, 60, 60), (85, 255, 255)))
            # Rojo: H 0-15 + 160-180
            red = cv2.countNonZero(
                cv2.bitwise_or(
                    cv2.inRange(hsv, (0, 60, 60), (15, 255, 255)),
                    cv2.inRange(hsv, (160, 60, 60), (180, 255, 255)),
                )
            )
            total = green + red
            if total < 200:
                return "neutral"
            ratio = green / total
            if ratio > 0.60:
                return "bullish"
            if ratio < 0.40:
                return "bearish"
            return "neutral"
        except ImportError:
            return self._detect_color_numpy(frame)
        except Exception:
            return self._detect_color_numpy(frame)

    @staticmethod
    def _detect_color_numpy(frame: np.ndarray) -> str:
        """Fallback sin OpenCV: compara canal R vs G en la mitad derecha."""
        try:
            h, w = frame.shape[:2]
            roi = frame[:, w // 2:, :]
            # frame puede ser BGR o RGB; comparamos posiciones 0 y 1
            ch0 = roi[:, :, 0].astype(float).mean()
            ch1 = roi[:, :, 1].astype(float).mean()
            diff = ch1 - ch0  # G - R en BGR, o R - G en RGB
            if diff > 8:
                return "bullish"
            if diff < -8:
                return "bearish"
            return "neutral"
        except Exception:
            return "unknown"

    # ── OCR ───────────────────────────────────────────────────────────────────

    def _run_ocr(self, frame: np.ndarray) -> List[str]:
        try:
            results = self._reader.readtext(frame, detail=0, paragraph=False)
            return [str(t).strip() for t in results if str(t).strip()]
        except Exception as exc:
            logger.debug("OCR readtext error: %s", exc)
            return []

    @staticmethod
    def _extract_prices(texts: List[str]) -> List[float]:
        prices: List[float] = []
        for text in texts:
            clean = text.replace(",", ".")
            for match in _PRICE_RE.finditer(clean):
                try:
                    val = float(match.group().replace(",", "."))
                    if 0.01 < val < 1_000_000:
                        prices.append(val)
                except ValueError:
                    pass
        # Deduplicar y ordenar
        seen: set[float] = set()
        dedup = []
        for p in prices:
            rounded = round(p, 4)
            if rounded not in seen:
                seen.add(rounded)
                dedup.append(rounded)
        return sorted(dedup)

    @staticmethod
    def _detect_pattern(texts: List[str], color: str) -> str:
        combined = " ".join(texts).lower()
        for kw, name in _PATTERN_MAP.items():
            if kw in combined:
                return name
        if any(k in combined for k in _BULLISH_KW):
            return "alcista"
        if any(k in combined for k in _BEARISH_KW):
            return "bajista"
        if color == "bullish":
            return "alcista"
        if color == "bearish":
            return "bajista"
        return "neutral"
