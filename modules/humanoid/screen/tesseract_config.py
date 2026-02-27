"""Configuración de ruta al binario Tesseract (Windows). Importar antes de usar pytesseract."""
from __future__ import annotations

import os
import sys

def set_tesseract_cmd() -> None:
    if sys.platform != "win32":
        return
    try:
        import pytesseract
        path = os.getenv("TESSERACT_CMD") or r"C:\Program Files\Tesseract-OCR\tesseract.exe"
        pytesseract.pytesseract.tesseract_cmd = path
    except Exception:
        pass
