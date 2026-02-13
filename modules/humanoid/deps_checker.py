"""Dependency checker: report missing_deps per module. No auto-install."""
from __future__ import annotations

import sqlite3
from typing import Any, Dict, List


def check_sqlite() -> Dict[str, Any]:
    try:
        sqlite3.connect(":memory:").close()
        return {"module": "sqlite", "available": True, "missing_deps": [], "suggested": ""}
    except Exception as e:
        return {"module": "sqlite", "available": False, "missing_deps": ["sqlite3"], "error": str(e), "suggested": "sqlite3 is usually built-in"}


def check_ollama() -> Dict[str, Any]:
    import os
    try:
        import httpx
        url = (os.getenv("OLLAMA_BASE_URL") or "http://127.0.0.1:11434") + "/api/tags"
        with httpx.Client(timeout=5) as client:
            r = client.get(url)
            return {"module": "ollama", "available": r.status_code == 200, "missing_deps": [] if r.status_code == 200 else ["ollama running"], "suggested": "Start Ollama service"}
    except Exception as e:
        return {"module": "ollama", "available": False, "missing_deps": ["ollama running"], "error": str(e), "suggested": "Start Ollama: ollama serve"}


def check_vision() -> Dict[str, Any]:
    """Vision deps: pillow, pytesseract (+ tesseract binary). Spec: tesseract, vision deps."""
    missing = []
    try:
        from PIL import Image
    except ImportError:
        missing.append("pillow")
    try:
        import pytesseract
        pytesseract.get_tesseract_version()
    except Exception:
        missing.append("pytesseract")
        missing.append("tesseract")
    return {"module": "vision", "available": len(missing) == 0, "missing_deps": missing, "suggested": "pip install pillow pytesseract; install Tesseract binary" if missing else ""}


def check_playwright() -> Dict[str, Any]:
    try:
        from modules.humanoid.web.navigator import get_missing_deps
        missing = get_missing_deps()
        return {"module": "web", "available": len(missing) == 0, "missing_deps": missing, "suggested": "pip install playwright && playwright install chromium"}
    except Exception as e:
        return {"module": "web", "available": False, "missing_deps": ["playwright"], "error": str(e), "suggested": "pip install playwright"}


def check_stt() -> Dict[str, Any]:
    """STT: whisper/faster-whisper. Spec: whisper si existe."""
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


def check_screen() -> Dict[str, Any]:
    """Screen intelligence: mss, pyautogui, pywinauto, pytesseract, pillow. No auto-install."""
    missing = []
    try:
        import mss
    except ImportError:
        missing.append("mss")
    try:
        import pyautogui
    except ImportError:
        missing.append("pyautogui")
    try:
        import pywinauto
    except ImportError:
        missing.append("pywinauto")
    try:
        from PIL import Image
    except ImportError:
        missing.append("pillow")
    try:
        import pytesseract
        pytesseract.get_tesseract_version()
    except Exception:
        missing.append("pytesseract")
        missing.append("tesseract")
    suggested = []
    if "mss" in missing or "pyautogui" in missing:
        suggested.append("pip install mss pyautogui")
    if "pywinauto" in missing:
        suggested.append("pip install pywinauto")
    if "pillow" in missing:
        suggested.append("pip install pillow")
    if "pytesseract" in missing or "tesseract" in missing:
        suggested.append("pip install pytesseract; install Tesseract binary")
    return {
        "module": "screen",
        "available": len(missing) == 0,
        "missing_deps": missing,
        "suggested": "; ".join(suggested) if suggested else "",
    }


def check_hardware() -> Dict[str, Any]:
    """RAM/GPU heuristics for model selection. No auto-install."""
    ram_gb = None
    low_ram = False
    try:
        import psutil
        mem = psutil.virtual_memory()
        ram_gb = round(mem.total / (1024 ** 3), 1)
        low_ram = mem.total < 8 * 1024 ** 3
    except ImportError:
        pass
    gpu = False
    try:
        import subprocess
        r = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True, timeout=3)
        gpu = r.returncode == 0 and bool(r.stdout)
    except Exception:
        pass
    return {
        "module": "hardware",
        "available": True,
        "ram_gb": ram_gb,
        "low_ram": low_ram,
        "gpu": gpu,
        "suggested": "Use AI_FAST_MODEL if low_ram" if low_ram else "",
    }


def check_all() -> Dict[str, Any]:
    """Return {ok, modules: [{module, available, missing_deps, suggested}], missing_deps: [...], suggested_commands: [...]}."""
    results = [check_sqlite(), check_ollama(), check_vision(), check_playwright(), check_stt(), check_tts(), check_screen(), check_hardware()]
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
