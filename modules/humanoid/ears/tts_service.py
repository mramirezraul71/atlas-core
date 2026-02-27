"""Text-to-speech service stub."""
from __future__ import annotations

from typing import Any, Dict


class TTSService:
    """Text-to-speech. Stub until real engine is integrated."""

    def speak(self, text: str, output_path: str | None = None) -> Dict[str, Any]:
        return {"ok": False, "error": "TTSService stub"}
