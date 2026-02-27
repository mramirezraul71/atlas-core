"""Vision service: OCR + image analyzer (stub)."""
from __future__ import annotations

from typing import Any, Dict


class VisionService:
    """OCR wrapper + image analyzer. OCR uses pytesseract if available."""

    def __init__(self) -> None:
        self._ocr_available = False
        try:
            from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
            set_tesseract_cmd()
            import pytesseract  # noqa: F401
            self._ocr_available = True
        except ImportError:
            pass

    def extract_text(self, image_path: str) -> Dict[str, Any]:
        if not self._ocr_available:
            return {"ok": False, "text": "", "error": "pytesseract not installed"}
        try:
            from modules.humanoid.screen.tesseract_config import set_tesseract_cmd
            set_tesseract_cmd()
            import pytesseract
            from PIL import Image
            text = pytesseract.image_to_string(Image.open(image_path))
            return {"ok": True, "text": text or "", "error": None}
        except Exception as e:
            return {"ok": False, "text": "", "error": str(e)}

    def analyze(self, image_path: str, task: str = "describe") -> Dict[str, Any]:
        """Stub for future vision model."""
        return {"ok": True, "result": {"task": task, "message": "VisionService stub"}, "error": None}
