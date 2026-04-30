"""TTS: deshabilitado pyttsx3 (voz femenina Windows). Solo browser speechSynthesis."""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.voice.tts")


def _check_deps() -> List[str]:
    return []


def is_available() -> bool:
    return False


def get_missing_deps() -> List[str]:
    return []


def speak(text: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """No-op: pyttsx3 deshabilitado. La voz de ATLAS es via browser speechSynthesis."""
    return {"ok": True, "message": "backend_tts_disabled", "engine": "browser_only"}
