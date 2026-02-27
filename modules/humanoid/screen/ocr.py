"""OCR: pytesseract if available, else empty text.

Incluye modo "data" para obtener bounding boxes (image_to_data) y habilitar click/move por texto.
Timeout configurable via ATLAS_OCR_TIMEOUT_SEC (default 15s) para evitar cuelgues.
"""
from __future__ import annotations

import io
import os
import concurrent.futures
from typing import Any, Dict, List, Optional, Tuple

from .status import _screen_deps_ok

# Timeout en segundos para cada llamada a pytesseract.
_OCR_TIMEOUT_SEC: float = float(os.getenv("ATLAS_OCR_TIMEOUT_SEC", "15") or 15)


def _run_tesseract_string(img) -> str:
    """Wrapper síncrono que corre image_to_string (potencialmente lento)."""
    import pytesseract
    return pytesseract.image_to_string(img)


def _run_tesseract_data(img) -> dict:
    """Wrapper síncrono que corre image_to_data (potencialmente lento)."""
    import pytesseract
    return pytesseract.image_to_data(img, output_type=pytesseract.Output.DICT)


def run_ocr(image_bytes: Optional[bytes] = None, image_path: Optional[str] = None, timeout_sec: Optional[float] = None) -> Tuple[str, str]:
    """
    Run OCR con timeout. Provide image_bytes o image_path. Returns (text, error).
    If pytesseract/tesseract not available, returns ("", "ocr_not_available").
    Si timeout, returns ("", "ocr_timeout").
    """
    if not _screen_deps_ok():
        return "", "screen_deps_missing"
    _timeout = timeout_sec if timeout_sec is not None else _OCR_TIMEOUT_SEC
    try:
        from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
        set_tesseract_cmd()
        from PIL import Image
        if image_bytes:
            img = Image.open(io.BytesIO(image_bytes))
        elif image_path:
            img = Image.open(image_path)
        else:
            return "", "no image"
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(_run_tesseract_string, img)
            try:
                text = future.result(timeout=_timeout)
            except concurrent.futures.TimeoutError:
                return "", "ocr_timeout"
        return (text or "").strip(), ""
    except ImportError:
        return "", "pytesseract_not_available"
    except Exception as e:
        return "", str(e)


def run_ocr_data(image_bytes: Optional[bytes] = None, image_path: Optional[str] = None, timeout_sec: Optional[float] = None) -> Tuple[List[Dict[str, Any]], str]:
    """
    Run OCR and return bounding boxes con timeout. Returns (items, error).
    item: {text, conf, bbox:[x,y,w,h]} in screen coordinates.
    """
    if not _screen_deps_ok():
        return [], "screen_deps_missing"
    _timeout = timeout_sec if timeout_sec is not None else _OCR_TIMEOUT_SEC
    try:
        from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
        set_tesseract_cmd()
        from PIL import Image
        if image_bytes:
            img = Image.open(io.BytesIO(image_bytes))
        elif image_path:
            img = Image.open(image_path)
        else:
            return [], "no image"
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(_run_tesseract_data, img)
            try:
                data = future.result(timeout=_timeout)
            except concurrent.futures.TimeoutError:
                return [], "ocr_timeout"
        items: List[Dict[str, Any]] = []
        n = len(data.get("text", []) or [])
        for i in range(n):
            txt = (data["text"][i] or "").strip()
            if not txt:
                continue
            try:
                conf = float(data.get("conf", ["-1"])[i])
            except Exception:
                conf = -1.0
            x = int(data.get("left", [0])[i] or 0)
            y = int(data.get("top", [0])[i] or 0)
            w = int(data.get("width", [0])[i] or 0)
            h = int(data.get("height", [0])[i] or 0)
            items.append({"text": txt, "conf": conf, "bbox": [x, y, w, h]})
        return items, ""
    except ImportError:
        return [], "pytesseract_not_available"
    except Exception as e:
        return [], str(e)
