"""Detección de rostros en imagen (OpenCV). Verificación opcional."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Optional

_log = logging.getLogger("humanoid.face.detector")

_missing: list = []


def _check_deps() -> list:
    global _missing
    if _missing and _missing != ["unknown"]:
        return _missing
    try:
        import cv2
        _missing = []
        return []
    except ImportError:
        _missing = ["opencv-python (pip install opencv-python)"]
        return _missing


def is_available() -> bool:
    return len(_check_deps()) == 0


def face_detect(image_path: str) -> Dict[str, Any]:
    """
    Detecta rostros en una imagen (ruta o path).
    Devuelve { ok, count, faces: [{x,y,w,h}], error }.
    """
    if not is_available():
        return {"ok": False, "count": 0, "faces": [], "error": "OpenCV no disponible", "missing_deps": _check_deps()}
    try:
        import cv2
        path = Path(image_path)
        if not path.exists():
            return {"ok": False, "count": 0, "faces": [], "error": f"Archivo no encontrado: {image_path}"}
        img = cv2.imread(str(path))
        if img is None:
            return {"ok": False, "count": 0, "faces": [], "error": "No se pudo leer la imagen"}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        if cascade.empty():
            return {"ok": False, "count": 0, "faces": [], "error": "Cascade classifier no cargado"}
        rects = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        faces = [{"x": int(x), "y": int(y), "w": int(w), "h": int(h)} for (x, y, w, h) in rects]
        return {"ok": True, "count": len(faces), "faces": faces, "error": None}
    except Exception as e:
        _log.exception("face_detect failed")
        return {"ok": False, "count": 0, "faces": [], "error": str(e)}


def face_check_image(image_data: bytes) -> Dict[str, Any]:
    """
    Detecta rostros a partir de bytes de imagen (PNG/JPEG).
    Devuelve { ok, faces_detected, message, count, error }.
    """
    if not is_available():
        return {"ok": False, "faces_detected": 0, "message": "Módulo no disponible", "count": 0, "error": "OpenCV no instalado"}
    try:
        import cv2
        import numpy as np
        arr = np.frombuffer(image_data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return {"ok": False, "faces_detected": 0, "message": "No se pudo decodificar la imagen", "count": 0, "error": "Formato no válido"}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        if cascade.empty():
            return {"ok": False, "faces_detected": 0, "message": "Error interno", "count": 0, "error": "Cascade no cargado"}
        rects = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        faces = [{"x": int(x), "y": int(y), "w": int(w), "h": int(h)} for (x, y, w, h) in rects]
        count = len(faces)
        return {
            "ok": True,
            "faces_detected": count,
            "message": "Rostro detectado" if count > 0 else "No se detectó rostro",
            "count": count,
            "faces": faces,
            "error": None,
        }
    except Exception as e:
        _log.exception("face_check_image failed")
        return {"ok": False, "faces_detected": 0, "message": "Error", "count": 0, "error": str(e)}
