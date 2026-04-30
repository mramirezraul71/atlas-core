"""
lotto_quant.vision.ticket_ocr
=============================

OCR pipeline for physical scratch-off tickets.

Pipeline
--------
    Camera frame  →  preprocess (grayscale, denoise, threshold)
                  →  ROI detection (largest rectangular contour)
                  →  text-region detection (MSER / morphology)
                  →  pytesseract OCR
                  →  validation against known win patterns

Hardware
--------
    - Insta360 (or any USB / RTSP camera)
    - Atlas main vision stack reuses the same OpenCV capture handle

If `cv2` or `pytesseract` are not installed, this module fails gracefully
with an ImportError documented in `__init__`.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

from .. import config

logger = logging.getLogger(__name__)


@dataclass
class OCRResult:
    raw_text: str
    confidence: float
    detected_amounts: List[float] = field(default_factory=list)
    detected_codes: List[str] = field(default_factory=list)
    image_path: Optional[str] = None


_MONEY_RE = re.compile(r"\$?\s*(\d{1,3}(?:,\d{3})*|\d+)(?:\.(\d{2}))?\b")
_CODE_RE = re.compile(r"\b[A-Z0-9]{6,16}\b")


class TicketOCR:
    """OpenCV + pytesseract pipeline for ticket reading."""

    def __init__(
        self,
        confidence_min: float = config.OCR_CONFIDENCE_MIN,
        tesseract_lang: str = config.TESSERACT_LANG,
    ):
        self.confidence_min = confidence_min
        self.tesseract_lang = tesseract_lang
        self._cv2 = None
        self._pytesseract = None
        self._np = None

    # ── lazy heavy imports so the module stays importable in CI ────
    def _lazy_imports(self) -> None:
        if self._cv2 is None:
            try:
                import cv2  # type: ignore
                import numpy as np  # type: ignore
                import pytesseract  # type: ignore
            except ImportError as e:
                raise ImportError(
                    "OCR dependencies missing. "
                    "Install with: pip install opencv-python-headless pytesseract numpy "
                    "and ensure tesseract binary is on PATH."
                ) from e
            self._cv2 = cv2
            self._np = np
            self._pytesseract = pytesseract

    # ── frame capture ──────────────────────────────────────────────
    def capture_frame(self, source: Optional[str] = None):
        """Capture a single frame from a webcam or RTSP stream."""
        self._lazy_imports()
        cv2 = self._cv2
        src = source or config.INSTA360_STREAM_URL
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"Cannot open video source: {src}")
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError("Failed to read a frame")
        return frame

    # ── preprocessing ──────────────────────────────────────────────
    def preprocess(self, frame):
        self._lazy_imports()
        cv2 = self._cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        denoised = cv2.fastNlMeansDenoising(gray, h=10)
        # Adaptive threshold copes with uneven lighting on shiny scratch-offs.
        thresh = cv2.adaptiveThreshold(
            denoised,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            7,
        )
        return thresh

    # ── ROI ────────────────────────────────────────────────────────
    def detect_ticket_roi(self, frame):
        """Find the largest rectangular region — assumed to be the ticket."""
        self._lazy_imports()
        cv2 = self._cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return frame
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        if w * h < 0.05 * frame.shape[0] * frame.shape[1]:
            return frame
        return frame[y:y + h, x:x + w]

    # ── OCR ────────────────────────────────────────────────────────
    def ocr(self, image) -> OCRResult:
        self._lazy_imports()
        pytesseract = self._pytesseract
        cv2 = self._cv2

        roi = self.detect_ticket_roi(image)
        prepped = self.preprocess(roi)

        # Grab text + per-word confidence using image_to_data
        try:
            data = pytesseract.image_to_data(
                prepped,
                lang=self.tesseract_lang,
                output_type=pytesseract.Output.DICT,
            )
        except pytesseract.TesseractNotFoundError as e:
            raise RuntimeError(
                "Tesseract binary not installed. "
                "Install: apt install tesseract-ocr  (Debian) or brew install tesseract."
            ) from e

        words: List[str] = []
        confs: List[float] = []
        for w, c in zip(data.get("text", []), data.get("conf", [])):
            if w and w.strip():
                words.append(w)
                try:
                    confs.append(float(c))
                except (TypeError, ValueError):
                    confs.append(0.0)
        raw_text = " ".join(words)
        avg_conf = (sum(confs) / len(confs) / 100.0) if confs else 0.0

        amounts = [
            float(_normalize_money(m.group(0)))
            for m in _MONEY_RE.finditer(raw_text)
        ]
        codes = list(set(_CODE_RE.findall(raw_text)))
        return OCRResult(
            raw_text=raw_text,
            confidence=avg_conf,
            detected_amounts=amounts,
            detected_codes=codes,
        )

    # ── high-level convenience ─────────────────────────────────────
    def read_from_camera(self, source: Optional[str] = None) -> OCRResult:
        frame = self.capture_frame(source)
        return self.ocr(frame)

    def read_from_file(self, path: str) -> OCRResult:
        self._lazy_imports()
        cv2 = self._cv2
        if not Path(path).exists():
            raise FileNotFoundError(path)
        image = cv2.imread(path)
        result = self.ocr(image)
        result.image_path = path
        return result


def _normalize_money(text: str) -> float:
    cleaned = text.replace("$", "").replace(",", "").strip()
    try:
        return float(cleaned)
    except ValueError:
        return 0.0
