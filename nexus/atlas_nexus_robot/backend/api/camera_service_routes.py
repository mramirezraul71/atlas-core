"""
Módulo de Cámara: servicio, detección, configuración.
API externa para ATLAS PUSH como "ojos externos".
"""

import os
import json
import subprocess
import sys
from typing import Optional
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.cameras.factory import get_camera_info, detect_cameras_list, release_camera
from vision.cameras.detector import save_active_config, load_active_config
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


class ConnectCameraBody(BaseModel):
    index: int = 0
    model: Optional[str] = None
    resolution: Optional[list] = None


class DisconnectCameraBody(BaseModel):
    index: Optional[int] = None


class AddRemoteCameraBody(BaseModel):
    """Cámara remota: fuera de la red local (dashcam en auto, etc.)."""
    url: str  # rtsp://host:port/path o http://host:port/path
    label: Optional[str] = None  # Nombre amigable (ej. "Dashcam frontal")
    connection_method: Optional[str] = "manual"  # manual, tailscale, zerotier, port_forward, other
    camera_type: Optional[str] = "auto"  # auto (dashcam), network


def _probe_camera_subprocess(index: int, width: int = 640, height: int = 480, timeout_s: int = 12) -> dict:
    """
    Prueba apertura y lectura de frame en subproceso aislado.
    Evita que fallos nativos de OpenCV derriben el proceso API.
    """
    code = """
import json
import cv2
import sys

idx = int(sys.argv[1])
w = int(sys.argv[2])
h = int(sys.argv[3])

result = {"opened": False, "frame_ok": False, "resolution": [w, h], "error": ""}
cap = None
try:
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if hasattr(cv2, "CAP_DSHOW") else cv2.VideoCapture(idx)
    if cap is not None and cap.isOpened():
        result["opened"] = True
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        ret, frame = cap.read()
        result["frame_ok"] = bool(ret and frame is not None)
        if frame is not None and hasattr(frame, "shape") and len(frame.shape) >= 2:
            result["resolution"] = [int(frame.shape[1]), int(frame.shape[0])]
    else:
        result["error"] = "open_failed"
except Exception as e:
    result["error"] = str(e)
finally:
    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass

print(json.dumps(result))
""".strip()
    try:
        p = subprocess.run(
            [sys.executable, "-X", "utf8", "-c", code, str(index), str(width), str(height)],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        if p.returncode != 0:
            return {"opened": False, "frame_ok": False, "resolution": [width, height], "error": (p.stderr or "").strip()[:300]}
        out = (p.stdout or "").strip()
        data = {}
        if out:
            # Algunos drivers (p.ej. Insta360/NexiGo) escriben logs en stdout.
            # Extraemos el último bloque JSON para evitar errores de parseo.
            try:
                data = json.loads(out)
            except Exception:
                parsed = None
                lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
                for ln in reversed(lines):
                    if ln.startswith("{") and ln.endswith("}"):
                        try:
                            parsed = json.loads(ln)
                            break
                        except Exception:
                            continue
                if parsed is None:
                    first = out.find("{")
                    last = out.rfind("}")
                    if first >= 0 and last > first:
                        chunk = out[first : last + 1]
                        try:
                            parsed = json.loads(chunk)
                        except Exception:
                            parsed = None
                if parsed is None:
                    return {
                        "opened": False,
                        "frame_ok": False,
                        "resolution": [width, height],
                        "error": "probe_output_parse_error: " + out[:180],
                    }
                data = parsed
        return {
            "opened": bool(data.get("opened")),
            "frame_ok": bool(data.get("frame_ok")),
            "resolution": data.get("resolution") or [width, height],
            "error": data.get("error", ""),
        }
    except subprocess.TimeoutExpired:
        return {"opened": False, "frame_ok": False, "resolution": [width, height], "error": "probe_timeout"}
    except Exception as e:
        return {"opened": False, "frame_ok": False, "resolution": [width, height], "error": str(e)}


def _connect_camera_index(index: int, model: str, resolution: list, capabilities: list, detected_count: int = 0) -> dict:
    width, height = 640, 480
    if isinstance(resolution, list) and len(resolution) >= 2:
        width, height = int(resolution[0]), int(resolution[1])
    probe = _probe_camera_subprocess(index=int(index), width=width, height=height)
    if probe.get("opened"):
        cfg = {
            "index": int(index),
            "model": model or "Webcam estándar",
            "resolution": probe.get("resolution") or [width, height],
            "capabilities": capabilities or ["video"],
        }
        save_active_config(cfg)
        return {
            "ok": True,
            "connected": True,
            "message": "Cámara conectada y respondiendo",
            "camera": {
                "model": cfg["model"],
                "driver": "uvc_standard",
                "resolution": cfg["resolution"],
                "capabilities": cfg["capabilities"],
                "index": int(index),
            },
            "frame_ok": bool(probe.get("frame_ok")),
            "detected_count": int(detected_count or 0),
        }
    return {
        "ok": False,
        "connected": False,
        "message": "No se pudo abrir la cámara",
        "detected_count": int(detected_count or 0),
        "probe_error": probe.get("error", ""),
        "camera": {"index": int(index)},
    }


@router.get("/service/status")
def camera_service_status(fast: bool = True):
    """
    Estado del módulo de cámara.
    Siempre incluye la lista de todas las cámaras instaladas (detected).
    """
    info = get_camera_info(fast=fast)
    base_url = os.getenv("NEXUS_BASE_URL", "http://localhost:8002")
    # `detect_cameras_list()` puede ser lento (escaneo OpenCV/PnP).
    # Para health/UX del dashboard, por defecto usamos fast=True.
    detected = [] if fast else detect_cameras_list()
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
        "fast": fast,
        "detected_hint": None if not fast else "Usa /api/camera/detect para escanear cámaras (puede tardar).",
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
        detected = detect_cameras_list()
        cfg = load_active_config() or {}
        idx = int(cfg.get("index", 0))
        if detected:
            idx = int(detected[0].get("index", idx))
        return _connect_camera_index(
            index=idx,
            model=cfg.get("model") or "Webcam estándar",
            resolution=cfg.get("resolution") or [640, 480],
            capabilities=cfg.get("capabilities") or ["video"],
            detected_count=len(detected),
        )
    except Exception as e:
        return {"ok": False, "connected": False, "error": str(e), "message": str(e)}


@router.post("/connect-index")
def camera_connect_index(body: ConnectCameraBody):
    """
    Conecta una cámara específica por índice.
    Útil para aislar fallos entre múltiples cámaras instaladas.
    """
    try:
        idx = int(body.index)
        if idx < 0:
            return {"ok": False, "connected": False, "error": "Índice inválido"}
        return _connect_camera_index(
            index=idx,
            model=body.model or "Webcam estándar",
            resolution=body.resolution or [640, 480],
            capabilities=["video"],
            detected_count=0,
        )
    except Exception as e:
        return {"ok": False, "connected": False, "error": str(e), "message": str(e)}


@router.post("/disconnect")
def camera_disconnect(body: Optional[DisconnectCameraBody] = None):
    """
    Apaga/desconecta la cámara activa del servicio.
    """
    try:
        release_camera()
        idx = None
        if body is not None and body.index is not None:
            idx = int(body.index)
        return {
            "ok": True,
            "disconnected": True,
            "message": "Cámara desconectada",
            "index": idx,
        }
    except Exception as e:
        return {"ok": False, "disconnected": False, "error": str(e), "message": str(e)}


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
