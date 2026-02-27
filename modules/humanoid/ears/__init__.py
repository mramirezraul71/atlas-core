"""Humanoid ears: STT + TTS."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .stt_service import STTService
from .tts_service import TTSService


class EarsModule(BaseModule, HealthCheckMixin):
    name = "ears"

    def __init__(self) -> None:
        self.stt = STTService()
        self.tts = TTSService()

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "message": "stub (STT/TTS disabled)",
            "details": {},
            "missing_deps": ["stt_engine", "tts_engine"],
        }


__all__ = ["EarsModule", "STTService", "TTSService"]
