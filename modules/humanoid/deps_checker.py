"""Dependency checker: report missing_deps per module. No auto-install."""
from __future__ import annotations

from typing import Any, Dict, List


def check_playwright() -> Dict[str, Any]:
    try:
        from modules.humanoid.web.navigator import get_missing_deps
        missing = get_missing_deps()
        return {"module": "web", "available": len(missing) == 0, "missing_deps": missing, "suggested": "pip install playwright && playwright install chromium"}
    except Exception as e:
        return {"module": "web", "available": False, "missing_deps": ["playwright"], "error": str(e), "suggested": "pip install playwright"}


def check_stt() -> Dict[str, Any]:
    try:
        from modules.humanoid.voice.stt import get_missing_deps
        missing = get_missing_deps()
        return {"module": "voice_stt", "available": len(missing) == 0, "missing_deps": missing, "suggested": "pip install faster-whisper"}
    except Exception as e:
        return {"module": "voice_stt", "available": False, "missing_deps": ["faster-whisper"], "error": str(e), "suggested": "pip install faster-whisper"}


def check_tts() -> Dict[str, Any]:
    try:
        from modules.humanoid.voice.tts import get_missing_deps
        missing = get_missing_deps()
        return {"module": "voice_tts", "available": len(missing) == 0, "missing_deps": missing, "suggested": "pip install pyttsx3"}
    except Exception as e:
        return {"module": "voice_tts", "available": False, "missing_deps": ["pyttsx3"], "error": str(e), "suggested": "pip install pyttsx3"}


def check_all() -> Dict[str, Any]:
    """Return {ok, modules: [{module, available, missing_deps, suggested}], missing_deps: [...], suggested_commands: [...]}."""
    results = [check_playwright(), check_stt(), check_tts()]
    all_missing: List[str] = []
    suggested: List[str] = []
    for r in results:
        all_missing.extend(r.get("missing_deps", []))
        if r.get("suggested"):
            suggested.append(r["suggested"])
    return {
        "ok": True,
        "modules": results,
        "missing_deps": list(dict.fromkeys(all_missing)),
        "suggested_commands": suggested,
    }
