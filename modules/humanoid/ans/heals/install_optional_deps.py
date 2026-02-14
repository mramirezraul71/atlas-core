"""Install optional pip deps from whitelist. Requires ANS_INSTALL_OPTIONAL_DEPS=true."""
from __future__ import annotations

import os
import subprocess
from .base import heal_result

_WHITELIST = frozenset({"pytesseract", "pywinauto", "pillow", "mss", "pyautogui", "playwright", "faster-whisper", "pyttsx3"})

# Comentarios breves por paquete (idioma prefijado)
_PKG_COMMENT = {
    "pytesseract": {"es": "OCR: lectura de texto en imágenes", "en": "OCR: read text from images"},
    "pywinauto": {"es": "Automatización de ventanas Windows", "en": "Windows UI automation"},
    "pillow": {"es": "Procesamiento de imágenes", "en": "Image processing"},
    "mss": {"es": "Captura de pantalla rápida", "en": "Fast screen capture"},
    "pyautogui": {"es": "Control de teclado y ratón", "en": "Keyboard and mouse control"},
    "playwright": {"es": "Automatización de navegadores web", "en": "Web browser automation"},
    "faster-whisper": {"es": "Transcripción de voz (STT)", "en": "Speech-to-text (STT)"},
    "pyttsx3": {"es": "Síntesis de voz (TTS)", "en": "Text-to-speech (TTS)"},
}


def _comment_for(packages: list, lang: str = "es") -> str:
    comments = [_PKG_COMMENT.get(p, {}).get(lang, p) for p in packages]
    return "; ".join(comments)


def run(**kwargs) -> dict:
    if os.getenv("ANS_INSTALL_OPTIONAL_DEPS", "").strip().lower() not in ("1", "true", "yes"):
        return heal_result(False, "install_optional_deps", "ANS_INSTALL_OPTIONAL_DEPS not enabled", {})
    try:
        from modules.humanoid.deps_checker import check_all
        missing = check_all().get("missing_deps", [])
    except Exception:
        missing = []
    if "tesseract" in missing:
        return heal_result(False, "install_optional_deps", "Falta el binario Tesseract en el OS; no usar pip. Ejecutar heal install_tesseract (winget).", {"skip_pip": True})
    to_install = [p for p in missing if p in _WHITELIST and p not in ("tesseract", "ollama running")]
    if not to_install:
        return heal_result(True, "install_optional_deps", "nothing to install (tesseract/ollama are system deps)", {})
    try:
        from modules.humanoid.ans.live_stream import emit
        comment_es = _comment_for(to_install, "es")
        comment_en = _comment_for(to_install, "en")
        emit("download_start", heal_id="install_optional_deps", message=f"pip install: {' '.join(to_install)}", details={"packages": to_install, "comment_es": comment_es, "comment_en": comment_en})
    except Exception:
        pass
    try:
        r = subprocess.run(
            [os.getenv("PYTHON", "python"), "-m", "pip", "install", "--quiet"] + to_install,
            capture_output=True,
            timeout=120,
            text=True,
        )
        ok = r.returncode == 0
        msg = "installed " + ", ".join(to_install) if ok else (r.stderr or r.stdout or "pip failed")[:200]
        try:
            from modules.humanoid.comms.scanner_store import record_download
            record_download(to_install, ok, msg, comment_es=_comment_for(to_install, "es"), comment_en=_comment_for(to_install, "en"))
        except Exception:
            pass
        try:
            from modules.humanoid.ans.live_stream import emit
            emit("download_end", heal_id="install_optional_deps", ok=ok, message=msg[:100], details={"installed": to_install, "comment_es": _comment_for(to_install, "es"), "comment_en": _comment_for(to_install, "en"), "stderr": (r.stderr or "")[:300] if not ok else None})
        except Exception:
            pass
        return heal_result(ok, "install_optional_deps", msg, {"installed": to_install})
    except Exception as e:
        return heal_result(False, "install_optional_deps", str(e), {}, str(e))
