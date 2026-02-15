"""OCR: pytesseract if available, else empty text.

Incluye modo "data" para obtener bounding boxes (image_to_data) y habilitar click/move por texto.
"""
from __future__ import annotations

import io
from typing import Any, Dict, List, Optional, Tuple

from .status import _screen_deps_ok


def run_ocr(image_bytes: Optional[bytes] = None, image_path: Optional[str] = None) -> Tuple[str, str]:
    """
    Run OCR. Provide image_bytes or image_path. Returns (text, error).
    If pytesseract/tesseract not available, returns ("", "ocr_not_available").
    """
    if not _screen_deps_ok():
        return "", "screen_deps_missing"
    try:
        from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
        set_tesseract_cmd()
        import pytesseract
        from PIL import Image
        if image_bytes:
            img = Image.open(io.BytesIO(image_bytes))
        elif image_path:
            img = Image.open(image_path)
        else:
            return "", "no image"
        text = pytesseract.image_to_string(img)
        return (text or "").strip(), ""
    except ImportError:
        return "", "pytesseract_not_available"
    except Exception as e:
        return "", str(e)


def run_ocr_data(image_bytes: Optional[bytes] = None, image_path: Optional[str] = None) -> Tuple[List[Dict[str, Any]], str]:
    """
    Run OCR and return bounding boxes. Returns (items, error).
    item: {text, conf, bbox:[x,y,w,h]} in screen coordinates.
    """
    if not _screen_deps_ok():
        return [], "screen_deps_missing"
    try:
        from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
        set_tesseract_cmd()
        import pytesseract
        from PIL import Image
        if image_bytes:
            img = Image.open(io.BytesIO(image_bytes))
        elif image_path:
            img = Image.open(image_path)
        else:
            return [], "no image"
        # output_type=DICT devuelve arrays paralelos.
        data = pytesseract.image_to_data(img, output_type=pytesseract.Output.DICT)
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
