"""Configuración de ruta al binario Tesseract (Windows). Importar antes de usar pytesseract."""
from __future__ import annotations

import os
import shutil
import sys


def set_tesseract_cmd() -> None:
    if sys.platform != "win32":
        return
    try:
        import pytesseract

        env_path = (os.getenv("TESSERACT_CMD") or "").strip()
        candidates = [
            env_path,
            r"C:\Program Files\Tesseract-OCR\tesseract.exe",
            r"C:\Program Files (x86)\Tesseract-OCR\tesseract.exe",
            shutil.which("tesseract") or "",
        ]
        for path in candidates:
            if path and os.path.exists(path):
                pytesseract.pytesseract.tesseract_cmd = path
                break
    except Exception:
        pass
