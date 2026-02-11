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
        import speech_recognition as sr
        _missing = []
        return []
    except ImportError:
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
                _missing = ["speech_recognition (pip install SpeechRecognition) o faster-whisper/whisper"]
                return _missing
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
