"""Visión Ubicua (Multidispositivo).

Capacidades:
- Descubrimiento local: RTSP (scan puertos) + ONVIF (WS-Discovery).
- Registro persistente en SQLite.
- Streaming Gateway: centraliza fuentes vía FFmpeg (HLS) y expone endpoints.
- Ojo activo: puede conmutar a una cámara de red al detectar movimiento.
"""

from __future__ import annotations

from .discovery import discover_local_cameras
from .motion import start_motion_watchdog, stop_motion_watchdog
from .registry import (ensure_db, get_camera, get_setting, list_cameras,
                       set_setting, upsert_camera)
from .streaming import (get_stream_paths, start_stream, stop_stream,
                        stream_status, take_snapshot)

__all__ = [
    "ensure_db",
    "discover_local_cameras",
    "list_cameras",
    "get_camera",
    "upsert_camera",
    "get_setting",
    "set_setting",
    "start_stream",
    "stop_stream",
    "stream_status",
    "take_snapshot",
    "get_stream_paths",
    "start_motion_watchdog",
    "stop_motion_watchdog",
]
