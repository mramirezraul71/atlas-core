"""OCR: pytesseract if available, else empty text."""
from __future__ import annotations

import io
from typing import Optional, Tuple

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
