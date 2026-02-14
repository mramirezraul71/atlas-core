"""
Descubridor de cámaras en red: prueba RTSP y HTTP MJPEG en candidatos.
"""

import json
import logging
import socket
import threading
from pathlib import Path
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

REGISTRY_PATH = Path(__file__).resolve().parent.parent.parent / "config" / "network_cameras.json"
_registry: List[Dict[str, Any]] = []
_lock = threading.Lock()


# Paths RTSP típicos de dashcams y cámaras duales (frontal/trasera)
DASHCAM_RTSP_PATHS = [
    ("stream1", "Dashcam frontal"),
    ("stream2", "Dashcam trasera"),
    ("livestream/11", "70mai principal"),
    ("livestream/12", "70mai secundario"),
    ("cam/realmonitor?channel=1&subtype=1", "Canal 1 frontal"),
    ("cam/realmonitor?channel=2&subtype=1", "Canal 2 trasera"),
    ("live/ch00_0", "Hikvision principal"),
    ("live/ch01_0", "Hikvision secundario"),
]


def _probe_rtsp_path(ip: str, port: int, path: str = "") -> bool:
    """Comprueba si un path RTSP responde."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        sock.connect((ip, port))
        req_path = path or "/"
        sock.send(f"OPTIONS rtsp://{ip}:{port}{req_path} RTSP/1.0\r\nCSeq: 1\r\n\r\n".encode())
        data = sock.recv(512).decode("utf-8", errors="ignore")
        sock.close()
        return "RTSP" in data.upper()
    except Exception:
        return False


def _probe_rtsp(ip: str, port: int) -> Optional[Dict[str, Any]]:
    """Prueba si el puerto responde a RTSP. Retorna una cámara genérica si responde en /."""
    if not _probe_rtsp_path(ip, port, "/"):
        return None
    url = f"rtsp://{ip}:{port}/"
    return {
        "id": f"rtsp_{ip.replace('.', '_')}_{port}",
        "ip": ip,
        "port": port,
        "protocol": "rtsp",
        "url": url,
        "model": f"Cámara IP {ip}",
        "source": "network",
        "type": "network",
    }


def _probe_http_mjpeg(ip: str, port: int) -> Optional[Dict[str, Any]]:
    """Prueba si hay stream MJPEG por HTTP."""
    try:
        import urllib.request
        base = f"http://{ip}:{port}"
        for path in ["/", "/video", "/stream", "/mjpeg", "/cam", "/api/camera/stream"]:
            try:
                req = urllib.request.Request(f"{base}{path}", method="GET")
                req.add_header("User-Agent", "ATLAS-NEXUS/1.0")
                with urllib.request.urlopen(req, timeout=3) as resp:
                    ct = resp.headers.get("Content-Type", "")
                    if "mjpeg" in ct.lower() or "multipart" in ct.lower() or "image" in ct.lower():
                        url = f"{base}{path}"
                        return {
                            "id": f"mjpeg_{ip.replace('.', '_')}_{port}",
                            "ip": ip,
                            "port": port,
                            "protocol": "mjpeg",
                            "url": url,
                            "model": f"Cámara IP {ip}",
                            "source": "network",
                            "type": "network",
                        }
            except Exception:
                continue
    except Exception as e:
        logger.debug("MJPEG probe %s:%s failed: %s", ip, port, e)
    return None


def _probe_dashcam_streams(ip: str, port: int) -> List[Dict[str, Any]]:
    """
    Para puerto 554 RTSP: prueba paths típicos de dashcams duales.
    Retorna lista de cámaras (frontal/trasera) si encuentra 2+ streams.
    """
    found = []
    for path, label in DASHCAM_RTSP_PATHS:
        path_norm = path if path.startswith("/") else f"/{path}"
        if _probe_rtsp_path(ip, port, path_norm):
            url = f"rtsp://{ip}:{port}{path_norm}"
            safe_path = path.replace("/", "_").replace("?", "_").replace("&", "_").replace("=", "_")[:25]
            cam_id = f"auto_{ip.replace('.', '_')}_{port}_{safe_path}"
            found.append({
                "id": cam_id,
                "ip": ip,
                "port": port,
                "protocol": "rtsp",
                "url": url,
                "model": f"{label} ({ip})",
                "source": "network",
                "type": "auto",
            })
    return found


def _probe_candidate(ip: str, port: int) -> Optional[Dict[str, Any]]:
    """Prueba un candidato (ip, port) y retorna info si es cámara."""
    if port == 554:
        return _probe_rtsp(ip, port)
    if port in (80, 8080, 8081, 8000, 443):
        return _probe_http_mjpeg(ip, port)
    return _probe_rtsp(ip, port) or _probe_http_mjpeg(ip, port)


def discover_network_cameras() -> List[Dict[str, Any]]:
    """
    Escanea la red y descubre cámaras (IP, dashcam dual frontal/trasera).
    Retorna lista de cámaras con id, ip, port, protocol, url, type (network|auto).
    """
    from .scanner import scan_network_for_camera_ports

    candidates = scan_network_for_camera_ports()
    found = []
    seen_ids = set()

    for ip, port in candidates:
        if port == 554:
            # RTSP: intentar primero paths de dashcam (frontal/trasera)
            dashcams = _probe_dashcam_streams(ip, port)
            if dashcams:
                for d in dashcams:
                    if d["id"] not in seen_ids:
                        seen_ids.add(d["id"])
                        found.append(d)
            else:
                cam = _probe_rtsp(ip, port)
                if cam and cam["id"] not in seen_ids:
                    seen_ids.add(cam["id"])
                    found.append(cam)
        else:
            cam = _probe_candidate(ip, port)
            if cam and cam.get("id") not in seen_ids:
                seen_ids.add(cam["id"])
                cam.setdefault("type", "network")
                found.append(cam)

    # Conservar cámaras remotas/manuales (no se pierden al escanear)
    existing = _load_registry()
    for c in existing:
        if c.get("source") in ("remote", "manual"):
            if c.get("id") not in seen_ids:
                seen_ids.add(c["id"])
                found.append(c)

    _save_registry(found)
    return found


def get_network_cameras() -> List[Dict[str, Any]]:
    """Obtiene cámaras de red del registro (sin escanear)."""
    return _load_registry()


def add_network_camera(cam: Dict[str, Any]) -> bool:
    """Añade una cámara manualmente al registro (local o remota)."""
    with _lock:
        registry = _load_registry()
        cam_id = cam.get("id") or _make_camera_id(cam)
        cam["id"] = cam_id
        cam.setdefault("source", "manual")
        registry = [c for c in registry if c.get("id") != cam_id]
        registry.append(cam)
        return _save_registry(registry)


def remove_network_camera(cam_id: str) -> bool:
    """Elimina una cámara del registro por ID."""
    with _lock:
        registry = _load_registry()
        registry = [c for c in registry if c.get("id") != cam_id]
        return _save_registry(registry)


def _make_camera_id(cam: Dict[str, Any]) -> str:
    """Genera ID único para cámara manual/remota."""
    url = cam.get("url", "")
    if url:
        import hashlib
        h = hashlib.md5(url.encode()).hexdigest()[:12]
        return f"remote_{h}"
    return f"manual_{cam.get('ip', '')}_{cam.get('port', 0)}".replace(".", "_")


def _load_registry() -> List[Dict[str, Any]]:
    if not REGISTRY_PATH.exists():
        return []
    try:
        with open(REGISTRY_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []


def _save_registry(data: List[Dict[str, Any]]) -> bool:
    try:
        REGISTRY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(REGISTRY_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return True
    except Exception as e:
        logger.error("Error saving network cameras: %s", e)
        return False
