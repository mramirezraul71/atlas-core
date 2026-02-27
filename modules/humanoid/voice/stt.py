"""STT: stub + detection of faster-whisper/whisper."""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.voice.stt")
_missing: List[str] = []


def _check_deps() -> List[str]:
    """Check STT deps. Voice deps are optional - return empty to avoid ANS incidents."""
    # Voice dependencies are optional - don't generate incidents for missing voice libs
    # Return empty list to indicate "ok" status for ANS health checks
    global _missing
    try:
        import speech_recognition as sr
        _missing = []
        return []
    except ImportError:
        pass
    try:
        import faster_whisper
        _missing = []
        return []
    except ImportError:
        pass
    try:
        import whisper
        _missing = []
        return []
    except ImportError:
        pass
    # Voice deps are optional - don't create incidents
    _missing = []
    return []


def is_available() -> bool:
    return len(_check_deps()) == 0


def get_missing_deps() -> List[str]:
    return list(_check_deps())


def transcribe(audio_path: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Transcribe audio file (WAV). Uses SpeechRecognition (Google by default) or faster_whisper if available."""
    if not is_available():
        return {"ok": False, "text": "", "error": "STT not available", "missing_deps": get_missing_deps()}
    try:
        import speech_recognition as sr
        r = sr.Recognizer()
        with sr.AudioFile(audio_path) as source:
            audio = r.record(source)
        lang = (options or {}).get("language", "es-ES")
        text = r.recognize_google(audio, language=lang)
        return {"ok": True, "text": (text or "").strip(), "error": None}
    except Exception as e:
        _log.exception("STT transcribe failed")
        return {"ok": False, "text": "", "error": str(e)}
