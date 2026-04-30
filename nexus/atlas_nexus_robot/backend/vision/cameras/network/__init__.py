"""
ATLAS NEXUS - Descubrimiento de cámaras en red WiFi.
Escaneo de LAN, detección RTSP/MJPEG, registro.
"""

from .discoverer import (add_network_camera, discover_network_cameras,
                         get_network_cameras, remove_network_camera)

__all__ = [
    "discover_network_cameras",
    "get_network_cameras",
    "add_network_camera",
    "remove_network_camera",
]
