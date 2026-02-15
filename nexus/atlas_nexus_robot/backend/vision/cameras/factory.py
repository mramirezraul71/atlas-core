"""
Factory de cámaras: obtiene la cámara activa según config o auto-detecta.
"""

import cv2
import logging
from typing import Optional, Dict, Any, List

from .standard_webcam import StandardWebcam
from .detector import detect_cameras, load_active_config, save_active_config
from .backend import preferred_backends

logger = logging.getLogger(__name__)

_camera_instance = None


def get_camera(force_detect: bool = False):
    """
    Retorna instancia de la cámara activa.
    Si hay config guardada, la usa; si no, auto-detecta y usa la primera disponible.
    """
    global _camera_instance

    config = load_active_config() if not force_detect else None

    if config:
        idx = config.get("index", 0)
        res = config.get("resolution", [640, 480])
        model = config.get("model", "Webcam estándar")
        backend = preferred_backends()[0]
        cam = StandardWebcam(
            index=idx,
            backend=backend,
            width=res[0] if isinstance(res, list) else 640,
            height=res[1] if isinstance(res, list) else 480,
            model_name=model,
        )
        if cam.open():
            _camera_instance = cam
            return cam
        cam.release()

    detected = detect_cameras()
    if not detected:
        cam = StandardWebcam(index=0, model_name="Webcam estándar")
        if cam.open():
            _camera_instance = cam
            return cam
        return None

    first = detected[0]
    idx = first.get("index", 0)
    res = first.get("resolution", [640, 480])
    model = first.get("model", "Webcam estándar")
    backend = preferred_backends()[0]

    cam = StandardWebcam(
        index=idx,
        backend=backend,
        width=res[0] if isinstance(res, list) else 640,
        height=res[1] if isinstance(res, list) else 480,
        model_name=model,
    )
    if cam.open():
        _camera_instance = cam
        save_active_config({
            "index": idx,
            "model": model,
            "resolution": res,
            "capabilities": first.get("capabilities", ["video"]),
        })
        return cam
    return None


def get_camera_info(fast: bool = True) -> Dict[str, Any]:
    """Info de la cámara activa para API externa (PUSH).

    `fast=True` evita auto-detección lenta cuando no hay cámara activa.
    """
    cam = _camera_instance
    if cam and cam.is_opened:
        return {
            "active": True,
            "properties": cam.get_properties(),
            "stream_url": "/api/vision/camera/stream",
            "external_eyes_url": "/api/vision/external/eyes",
        }
    if fast:
        detected = []
    else:
        detected = detect_cameras()
    return {
        "active": False,
        "detected": detected,
        "stream_url": "/api/vision/camera/stream",
        "external_eyes_url": "/api/vision/external/eyes",
    }


def detect_cameras_list() -> List[Dict[str, Any]]:
    """Wrapper para usar desde api."""
    return detect_cameras()
