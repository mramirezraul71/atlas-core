"""
Módulo de Cámara: servicio, detección, configuración.
API externa para ATLAS PUSH como "ojos externos".
"""

import os
from typing import Optional
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.cameras.factory import get_camera, get_camera_info, detect_cameras_list
from vision.cameras.detector import save_active_config
from vision.cameras.network import (
    discover_network_cameras,
    get_network_cameras,
    add_network_camera,
    remove_network_camera,
)

router = APIRouter(prefix="/api/camera", tags=["camera"])


class ConfigureCameraBody(BaseModel):
    index: Optional[int] = None
    model: Optional[str] = None
    resolution: Optional[list] = None


class AddRemoteCameraBody(BaseModel):
    """Cámara remota: fuera de la red local (dashcam en auto, etc.)."""
    url: str  # rtsp://host:port/path o http://host:port/path
    label: Optional[str] = None  # Nombre amigable (ej. "Dashcam frontal")
    connection_method: Optional[str] = "manual"  # manual, tailscale, zerotier, port_forward, other
    camera_type: Optional[str] = "auto"  # auto (dashcam), network


@router.get("/service/status")
def camera_service_status():
    """
    Estado del módulo de cámara.
    Siempre incluye la lista de todas las cámaras instaladas (detected).
    """
    info = get_camera_info()
    base_url = os.getenv("NEXUS_BASE_URL", "http://localhost:8002")
    detected = detect_cameras_list()
    network_cameras = get_network_cameras()
    return {
        "ok": True,
        "service": "camera",
        "active": info.get("active", False),
        "camera": info.get("properties", {}) if info.get("active") else None,
        "detected": detected,
        "network_cameras": network_cameras,
        "stream_url": f"{base_url}{info.get('stream_url', '/api/vision/camera/stream')}",
        "screen_stream_url": f"{base_url}/api/vision/screen/stream",
        "external_eyes_url": f"{base_url}{info.get('external_eyes_url', '/api/vision/external/eyes')}",
    }


@router.get("/detect")
def camera_detect():
    """
    Detecta cámaras disponibles (USB/Windows PnP + OpenCV).
    El robot identifica automáticamente; el usuario puede confirmar.
    """
    cameras = detect_cameras_list()
    return {"ok": True, "cameras": cameras, "count": len(cameras)}


@router.post("/connect")
def camera_connect():
    """
    Intenta conectar/activar la cámara: detecta, abre y verifica lectura.
    Útil cuando la cámara no responde; fuerza re-detección.
    """
    try:
        from vision.cameras.factory import get_camera, get_camera_info, detect_cameras_list
        cam = get_camera(force_detect=True)
        info = get_camera_info()
        detected = detect_cameras_list()
        if cam and cam.is_opened:
            # Probar lectura
            ret, frame = cam.read()
            return {
                "ok": True,
                "connected": True,
                "message": "Cámara conectada y respondiendo",
                "camera": info.get("properties", {}),
                "frame_ok": ret and frame is not None,
                "detected_count": len(detected),
            }
        return {
            "ok": False,
            "connected": False,
            "message": "No se pudo abrir la cámara",
            "detected": detected,
            "detected_count": len(detected),
        }
    except Exception as e:
        return {"ok": False, "connected": False, "error": str(e), "message": str(e)}


@router.post("/configure")
def camera_configure(body: ConfigureCameraBody):
    """
    Configura la cámara activa.
    Si el usuario tiene la info, la suministra; si no, se usa la auto-detectada.
    """
    if body.index is not None:
        config = {
            "index": body.index,
            "model": body.model or "Webcam estándar",
            "resolution": body.resolution or [640, 480],
            "capabilities": ["video"],
        }
        if save_active_config(config):
            get_camera(force_detect=True)
            return {"ok": True, "message": "Cámara configurada", "config": config}
    return {"ok": False, "error": "Se requiere al menos 'index' para configurar"}


@router.get("/network/scan")
def network_camera_scan():
    """
    Escanea la red WiFi local y descubre cámaras IP (RTSP, MJPEG).
    Usa la red detectada por Windows.
    """
    try:
        cameras = discover_network_cameras()
        return {"ok": True, "cameras": cameras, "count": len(cameras)}
    except Exception as e:
        return {"ok": False, "cameras": [], "count": 0, "error": str(e)}


CONNECTION_METHODS = [
    {
        "id": "not_sure",
        "name": "No sé — conectar una vez para registrar",
        "description": "Solo quieres que funcione. Introduce la URL, prueba la conexión y guarda. El método se registrará automáticamente.",
        "url_hint": "Pega la URL que te dé la app o el fabricante de la dashcam.",
    },
    {
        "id": "tailscale",
        "name": "Tailscale",
        "description": "Red mesh VPN. Instala Tailscale en router/raspberry del auto o en el dispositivo que expone la dashcam.",
        "url_hint": "rtsp://nombre-máquina.tail12345.ts.net:554/stream1",
    },
    {
        "id": "zerotier",
        "name": "ZeroTier",
        "description": "Red virtual privada. Une dashcam y robot en la misma red ZeroTier.",
        "url_hint": "rtsp://10.x.x.x:554/stream1 (IP de ZeroTier)",
    },
    {
        "id": "port_forward",
        "name": "Port Forwarding",
        "description": "Reenvío de puertos en router. Requiere IP pública o DDNS.",
        "url_hint": "rtsp://tu-ip-o-dominio:554/stream1",
    },
    {
        "id": "manual",
        "name": "URL manual",
        "description": "Introduce la URL completa si ya tienes acceso (cloud, túnel propio, etc.).",
        "url_hint": "rtsp://host:puerto/ruta",
    },
    {
        "id": "other",
        "name": "Otro",
        "description": "WireGuard, OpenVPN, túnel SSH o método personalizado.",
        "url_hint": "Cualquier URL válida rtsp:// o http://",
    },
]


@router.get("/network/connection-methods")
def get_connection_methods():
    """Opciones de conexión para cámaras remotas (fuera de red local)."""
    return {"ok": True, "methods": CONNECTION_METHODS}


@router.get("/network/list")
def network_camera_list():
    """Lista cámaras de red ya descubiertas (sin escanear)."""
    cameras = get_network_cameras()
    return {"ok": True, "cameras": cameras, "count": len(cameras)}


@router.get("/network/test-url")
def test_camera_url(url: str = ""):
    """
    Prueba si una URL de stream (rtsp/http) es accesible.
    Abre el stream, lee un frame y retorna ok/error.
    Para que el usuario conecte una vez y verifique antes de guardar.
    """
    raw = (url or "").strip()
    if not raw or not (raw.startswith("rtsp://") or raw.startswith("http://") or raw.startswith("https://")):
        return {"ok": False, "error": "URL inválida. Usa rtsp:// o http://"}

    try:
        import cv2
        cap = cv2.VideoCapture(raw)
        if not cap.isOpened():
            return {"ok": False, "error": "No se pudo abrir el stream"}
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            return {"ok": False, "error": "No se pudo leer un frame"}
        return {"ok": True, "message": "Conexión exitosa"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/network/add-remote")
def add_remote_camera(body: AddRemoteCameraBody):
    """
    Añade una cámara REMOTA (fuera de tu red local).
    Para dashcams en el auto, uso con VPN (Tailscale/ZeroTier) o URL manual.

    - url: rtsp://host:puerto/ruta o http://host:puerto/ruta
    - connection_method: tailscale | zerotier | port_forward | manual | other
    """
    from urllib.parse import urlparse

    raw = (body.url or "").strip()
    if not raw or not (raw.startswith("rtsp://") or raw.startswith("http://") or raw.startswith("https://")):
        raise HTTPException(
            400,
            "URL inválida. Usa rtsp://host:puerto/ruta o http://host:puerto/ruta",
        )

    parsed = urlparse(raw)
    host = parsed.hostname or "unknown"
    port = parsed.port or (554 if parsed.scheme == "rtsp" else 80)
    label = (body.label or "").strip() or f"{host}:{port}"

    cam = {
        "url": raw,
        "ip": host,
        "port": port,
        "protocol": "rtsp" if "rtsp" in parsed.scheme else "mjpeg",
        "model": label,
        "source": "remote",
        "type": body.camera_type or "auto",
        "connection_method": (body.connection_method or "manual").replace("not_sure", "registered"),
    }
    if add_network_camera(cam):
        return {"ok": True, "message": "Cámara remota añadida", "camera": cam}
    raise HTTPException(500, "No se pudo guardar la cámara")


@router.delete("/network/remove/{cam_id}")
def remove_remote_camera(cam_id: str):
    """Elimina una cámara de red/remota del registro."""
    if remove_network_camera(cam_id):
        return {"ok": True, "message": "Cámara eliminada"}
    return {"ok": False, "error": "ID no encontrado"}


# Nota: El endpoint /api/vision/external/eyes (ojos externos para PUSH) está en vision_routes.
