"""
ATLAS NEXUS - Módulo de Cámaras
Detección automática (USB/Windows), registry, factory.
Expone API para ATLAS PUSH como "ojos externos".
"""

from .base import CameraBase
from .detector import detect_cameras
from .factory import detect_cameras_list, get_camera, get_camera_info

__all__ = [
    "CameraBase",
    "get_camera",
    "get_camera_info",
    "detect_cameras",
    "detect_cameras_list",
]
