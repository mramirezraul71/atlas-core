"""STT: stub + detection of faster-whisper/whisper."""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.voice.stt")
_missing: List[str] = []


def _check_deps() -> List[str]:
    global _missing
    if _missing and _missing != ["unknown"]:
        return _missing
    try:
        import faster_whisper
        _missing = []
        return []
    except ImportError:
        try:
            import whisper
            _missing = []
            return []
        except ImportError:
            _missing = ["faster-whisper or openai-whisper"]
            return _missing
    _missing = ["faster-whisper or openai-whisper"]
    return _missing


def is_available() -> bool:
    return len(_check_deps()) == 0


def get_missing_deps() -> List[str]:
    return list(_check_deps())


def transcribe(audio_path: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Transcribe audio file. Stub: returns not_available if deps missing."""
    if not is_available():
        return {"ok": False, "text": "", "error": "STT not available", "missing_deps": get_missing_deps()}
    return {"ok": False, "text": "", "error": "transcribe not implemented (stub)"}
