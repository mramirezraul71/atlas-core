"""
Detección de cámaras: USB/Windows PnP + OpenCV.
Prioridad: Windows detecta al conectar USB → enumeramos y matcheamos con registry.
"""

import json
import os
import subprocess
import sys
import cv2
import logging
from pathlib import Path
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

REGISTRY_PATH = Path(__file__).resolve().parent.parent / "registry" / "camera_registry.json"
CONFIG_PATH = Path(__file__).resolve().parent.parent.parent / "config" / "active_camera.json"


def _load_registry() -> Dict[str, Any]:
    """Carga el registry de cámaras conocidas."""
    try:
        with open(REGISTRY_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        logger.warning("No se pudo cargar camera_registry: %s", e)
        return {"cameras": []}


def _detect_windows_pnp() -> List[Dict[str, str]]:
    """En Windows: enumera dispositivos de cámara vía PowerShell (PnP)."""
    devices = []
    if sys.platform != "win32":
        return devices
    try:
        ps = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                "$cam=(Get-PnpDevice -Class Camera -EA SilentlyContinue);"
                "$img=(Get-PnpDevice -Class Image -EA SilentlyContinue);"
                "($cam+$img|Select-Object Status,FriendlyName,InstanceId -Unique) | ConvertTo-Json -Compress",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if ps.returncode == 0 and ps.stdout:
            data = json.loads(ps.stdout)
            if isinstance(data, dict):
                data = [data]
            for d in data:
                name = (d.get("FriendlyName") or "").strip()
                if name and d.get("Status") == "OK":
                    vid_pid = ""
                    instance = d.get("InstanceId") or ""
                    if "VID_" in instance and "PID_" in instance:
                        parts = instance.split("\\")
                        for p in parts:
                            if "VID_" in p:
                                vid_pid = p
                                break
                    devices.append({
                        "name": name,
                        "instance_id": instance,
                        "vid_pid": vid_pid,
                    })
    except Exception as e:
        logger.debug("Windows PnP detection failed: %s", e)
    return devices


def _detect_opencv_indices() -> List[Dict[str, Any]]:
    """Enumerar índices de cámara que OpenCV puede abrir (0-9, múltiples backends)."""
    seen = {}
    if sys.platform == "win32":
        backends = []
        if hasattr(cv2, "CAP_DSHOW"):
            backends.append(cv2.CAP_DSHOW)
        if hasattr(cv2, "CAP_MSMF"):
            backends.append(cv2.CAP_MSMF)
        backends.append(cv2.CAP_ANY)
    else:
        backends = [cv2.CAP_ANY]
    for backend in backends:
        for i in range(10):
            if i in seen:
                continue
            try:
                cap = cv2.VideoCapture(i, backend)
                if cap.isOpened():
                    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                    cap.release()
                    seen[i] = {
                        "index": i,
                        "backend": int(backend),
                        "resolution": [w or 640, h or 480],
                    }
            except Exception:
                pass
    return [seen[k] for k in sorted(seen)]


def _match_to_registry(device_name: str, vid_pid: str = "") -> Optional[Dict[str, Any]]:
    """Busca coincidencia en el registry por nombre, keywords o VID."""
    registry = _load_registry()
    device_lower = device_name.lower()
    vid_upper = (vid_pid or "").upper().replace("0X", "").replace("0x", "")

    for cam in registry.get("cameras", []):
        name = (cam.get("name") or "").lower()
        if name in device_lower or device_lower in name:
            return cam
        for kw in cam.get("keywords", []):
            if (kw or "").lower() in device_lower:
                return cam
        for vid in cam.get("vendor_ids", []):
            v = vid.upper().replace("0X", "").replace("0x", "").replace("VID_", "")
            if v in vid_upper or f"VID_{v}" in vid_upper:
                return cam
    return None


def detect_cameras() -> List[Dict[str, Any]]:
    """
    Detecta cámaras disponibles.
    Prioridad: Windows PnP (al conectar USB) + OpenCV indices.
    Retorna lista con: index, model, driver, capabilities, resolution.
    """
    registry = _load_registry()
    pnp_devices = _detect_windows_pnp()
    cv_indices = _detect_opencv_indices()

    result = []
    seen_indices = set()

    for pnp in pnp_devices:
        name = pnp.get("name", "")
        vid_pid = pnp.get("vid_pid", "")
        matched = _match_to_registry(name, vid_pid)

        params = (matched or {}).get("params", {})
        res = params.get("resolution", [640, 480])
        driver = (matched or {}).get("driver", "uvc_standard")
        caps = (matched or {}).get("capabilities", ["video"])

        for cv_info in cv_indices:
            idx = cv_info.get("index", 0)
            if idx in seen_indices:
                continue
            seen_indices.add(idx)
            result.append({
                "index": idx,
                "model": matched.get("name", name) if matched else name,
                "driver": driver,
                "capabilities": caps,
                "resolution": res,
                "source": "pnp",
                "vid_pid": vid_pid,
            })
            break
        else:
            if cv_indices and 0 not in seen_indices:
                idx = cv_indices[0]["index"]
                seen_indices.add(idx)
                result.append({
                    "index": idx,
                    "model": matched.get("name", name) if matched else name,
                    "driver": driver,
                    "capabilities": caps,
                    "resolution": res,
                    "source": "pnp",
                    "vid_pid": vid_pid,
                })

    for cv_info in cv_indices:
        idx = cv_info.get("index")
        if idx in seen_indices:
            continue
        seen_indices.add(idx)
        result.append({
            "index": idx,
            "model": "Webcam estándar",
            "driver": "uvc_standard",
            "capabilities": ["video"],
            "resolution": cv_info.get("resolution", [640, 480]),
            "source": "opencv",
        })

    return result


def save_active_config(camera_info: Dict[str, Any]) -> bool:
    """Guarda la cámara activa en config para que el factory la use."""
    try:
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(camera_info, f, indent=2)
        return True
    except Exception as e:
        logger.error("Error guardando active_camera: %s", e)
        return False


def load_active_config() -> Optional[Dict[str, Any]]:
    """Carga la config activa de cámara."""
    if not CONFIG_PATH.exists():
        return None
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None
