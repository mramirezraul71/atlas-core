"""Speech-to-text service stub."""
from __future__ import annotations

from typing import Any, Dict


class STTService:
    """Speech-to-text. Stub until real engine (e.g. Whisper) is integrated."""

    def transcribe(self, audio_path: str) -> Dict[str, Any]:
        return {"ok": False, "text": "", "error": "STTService stub"}
