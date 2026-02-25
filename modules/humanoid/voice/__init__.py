"""Voice: STT/TTS stubs + deps detection."""
from __future__ import annotations

from .controller import voice_listen_stub, voice_speak, voice_status
from .stt import get_missing_deps as stt_missing_deps
from .stt import is_available as stt_available
from .tts import get_missing_deps as tts_missing_deps
from .tts import is_available as tts_available

__all__ = [
    "voice_status",
    "voice_speak",
    "voice_listen_stub",
    "stt_available",
    "tts_available",
    "stt_missing_deps",
    "tts_missing_deps",
]
