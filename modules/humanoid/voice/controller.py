"""Voice controller: endpoints helpers for speak, listen, status.

Modo seguro: la voz NO ejecuta comandos sin confirmación explícita.
Cualquier integración que traduzca voz → comando debe pedir confirmación antes de ejecutar."""
from __future__ import annotations

from typing import Any, Dict

from .stt import get_missing_deps as stt_missing, is_available as stt_available
from .tts import get_missing_deps as tts_missing, is_available as tts_available


def voice_status() -> Dict[str, Any]:
    return {
        "enabled": stt_available() or tts_available(),
        "stt": {"available": stt_available(), "missing_deps": stt_missing()},
        "tts": {"available": tts_available(), "missing_deps": tts_missing()},
    }


def voice_speak(text: str) -> Dict[str, Any]:
    from .tts import speak
    return speak(text)


def voice_listen_stub() -> Dict[str, Any]:
    """Stub: listen not implemented."""
    return {"ok": True, "text": "", "error": None, "message": "listen is a stub (STT required)"}
