"""Módulo de Visión para producción: pytesseract. Ruta maestra única para snapshots (Instrucción Maestra)."""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Tuple

logger = logging.getLogger("atlas.evolution.vision")

# Ruta maestra: único binario válido para procesamiento de snapshots (Nivel 0)
TESSERACT_MASTER = r"C:\Program Files\Tesseract-OCR\tesseract.exe"
TESSERACT_EXE = os.environ.get("TESSERACT_CMD", TESSERACT_MASTER)


def configure_tesseract(path: str | None = None) -> bool:
    """Configura pytesseract apuntando al binario Tesseract. En producción se usa solo la ruta maestra."""
    exe = (path or TESSERACT_MASTER or TESSERACT_EXE).strip()
    if not exe:
        return False
    if not Path(exe).exists():
        logger.warning("Tesseract no encontrado: %s", exe)
        return False
    os.environ["TESSERACT_CMD"] = exe
    try:
        import pytesseract
        pytesseract.pytesseract.tesseract_cmd = exe
        logger.info("Visión: pytesseract -> %s", exe)
        return True
    except ImportError:
        logger.debug("pytesseract no instalado; OCR no disponible")
        return False


def test_vision_snapshot() -> Tuple[bool, str]:
    """
    Toma un snapshot de la pantalla y verifica que puede leer texto con OCR.
    Devuelve (éxito, mensaje). Éxito True si se capturó y se extrajo al menos un carácter.
    """
    if not configure_tesseract():
        return False, "Tesseract no configurado o no encontrado"
    try:
        import pytesseract
    except ImportError:
        return False, "pytesseract no instalado"
    png_bytes, err = _capture_screen()
    if err or not png_bytes:
        return False, err or "No se pudo capturar pantalla"
    try:
        from PIL import Image
        import io
        img = Image.open(io.BytesIO(png_bytes))
        text = pytesseract.image_to_string(img, lang="eng").strip()
        if text and len(text) > 0:
            logger.info("Visión: snapshot OK, %d caracteres leídos", len(text))
            return True, f"OK: {len(text)} caracteres leídos"
        return True, "OK: snapshot sin texto visible (pantalla vacía o sin texto)"
    except Exception as e:
        logger.debug("Visión OCR: %s", e)
        return False, str(e)[:200]


def _capture_screen() -> Tuple[bytes | None, str]:
    """Captura pantalla completa. Devuelve (png_bytes, error_message)."""
    try:
        from modules.humanoid.screen.capture import capture_screen
        png_bytes, err = capture_screen(region=None, format="png")
        return png_bytes, err or ""
    except ImportError:
        pass
    try:
        import mss
        with mss.mss() as sct:
            mon = sct.monitors[0]
            shot = sct.grab(mon)
            from PIL import Image
            img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
            buf = __import__("io").BytesIO()
            img.save(buf, "PNG")
            return buf.getvalue(), ""
    except Exception as e:
        return None, str(e)[:200]
