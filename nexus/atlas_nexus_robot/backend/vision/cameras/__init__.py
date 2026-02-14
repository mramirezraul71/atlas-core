"""
ATLAS NEXUS - Módulo de Cámaras
Detección automática (USB/Windows), registry, factory.
Expone API para ATLAS PUSH como "ojos externos".
"""

from .base import CameraBase
from .factory import get_camera, get_camera_info, detect_cameras_list
from .detector import detect_cameras

__all__ = ["CameraBase", "get_camera", "get_camera_info", "detect_cameras", "detect_cameras_list"]
