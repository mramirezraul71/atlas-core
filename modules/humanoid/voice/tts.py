"""TTS: stub + detection of piper/sherpa-onnx or system."""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.voice.tts")
_missing: List[str] = []


def _check_deps() -> List[str]:
    global _missing
    if _missing and _missing != ["unknown"]:
        return _missing
    try:
        import pyttsx3
        _missing = []
        return []
    except ImportError:
        _missing = ["pyttsx3 (or piper/sherpa-onnx)"]
        return _missing
    _missing = ["pyttsx3 or piper"]
    return _missing


def is_available() -> bool:
    return len(_check_deps()) == 0


def get_missing_deps() -> List[str]:
    return list(_check_deps())


def speak(text: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Speak text. Stub: returns not_available if deps missing."""
    if not is_available():
        return {"ok": False, "error": "TTS not available", "missing_deps": get_missing_deps()}
    return {"ok": False, "error": "speak not implemented (stub)"}
