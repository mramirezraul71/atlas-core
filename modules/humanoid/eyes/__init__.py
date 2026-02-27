"""Humanoid eyes: vision service."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .vision_service import VisionService


class EyesModule(BaseModule, HealthCheckMixin):
    name = "eyes"

    def __init__(self) -> None:
        self.vision = VisionService()

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        missing = [] if self.vision._ocr_available else ["pytesseract", "Pillow"]
        return {
            "ok": True,
            "message": "ok" if self.vision._ocr_available else "vision disabled (missing deps)",
            "details": {"ocr_available": self.vision._ocr_available},
            "missing_deps": missing,
        }

    def info(self) -> Dict[str, Any]:
        return {"module": self.name, "ocr_available": self.vision._ocr_available}


__all__ = ["EyesModule", "VisionService"]
