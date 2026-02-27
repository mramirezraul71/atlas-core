"""Visión Ubicua (Multidispositivo).

Capacidades:
- Descubrimiento local: RTSP (scan puertos) + ONVIF (WS-Discovery).
- Registro persistente en SQLite.
- Streaming Gateway: centraliza fuentes vía FFmpeg (HLS) y expone endpoints.
- Ojo activo: puede conmutar a una cámara de red al detectar movimiento.
"""

from __future__ import annotations

from .registry import (
    ensure_db,
    list_cameras,
    get_camera,
    upsert_camera,
    get_setting,
    set_setting,
)
from .discovery import discover_local_cameras
from .streaming import (
    start_stream,
    stop_stream,
    stream_status,
    take_snapshot,
    get_stream_paths,
)
from .motion import start_motion_watchdog, stop_motion_watchdog

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

