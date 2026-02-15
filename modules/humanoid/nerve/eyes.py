"""Nervio: ojos. Local (hands_eyes) o Nexus (Robot 8002 snapshot). El nervio llega hasta Nexus."""
from __future__ import annotations

import base64
import os
import urllib.request
from typing import Any, Dict, Optional

NEXUS_ROBOT_API_URL = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
NEXUS_ENABLED = (os.getenv("NEXUS_ENABLED") or "").strip().lower() in ("1", "true", "yes", "y", "on")
NERVE_NEXUS_TIMEOUT = int(os.getenv("NERVE_NEXUS_TIMEOUT", "5"))


def _nexus_eyes_available() -> bool:
    if not NEXUS_ENABLED:
        return False
    try:
        req = urllib.request.Request(
            f"{NEXUS_ROBOT_API_URL}/api/vision/external/eyes",
            method="GET",
            headers={"Accept": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=NERVE_NEXUS_TIMEOUT) as r:
            if r.status != 200:
                return False
            import json
            data = json.loads(r.read().decode("utf-8"))
            return bool(data.get("ok", False) and data.get("snapshot_url"))
    except Exception:
        return False


def _nexus_snapshot(source: str = "screen") -> Optional[bytes]:
    """Obtiene un frame desde Nexus (Robot). source=camera|screen."""
    url = f"{NEXUS_ROBOT_API_URL}/api/vision/snapshot?source={source}"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=NERVE_NEXUS_TIMEOUT) as r:
            if r.status != 200:
                return None
            return r.read()
    except Exception:
        return None


def _nexus_snapshot_advanced(
    source: str = "screen",
    enhance: str = "auto",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
    index: int = 0,
) -> Optional[bytes]:
    """
    Snapshot avanzado: permite control fino de "ojos externos" (nitidez + foco/zoom digital).
    """
    try:
        from urllib.parse import urlencode

        qs = urlencode({
            "source": source,
            "enhance": enhance,
            "jpeg_quality": int(jpeg_quality),
            "focus_x": float(focus_x),
            "focus_y": float(focus_y),
            "zoom": float(zoom),
            "index": int(index),
        })
        url = f"{NEXUS_ROBOT_API_URL}/api/vision/snapshot?{qs}"
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=NERVE_NEXUS_TIMEOUT) as r:
            if r.status != 200:
                return None
            return r.read()
    except Exception:
        return None


def eyes_capture(
    use_nexus_if_available: bool = True,
    source: str = "screen",
    region: Optional[tuple] = None,
    enhance: str = "auto",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
    index: int = 0,
    eye: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Captura lo que ven los ojos: local (pantalla) o Nexus (ojos externos).
    use_nexus_if_available=True y Nexus conectado → snapshot desde Robot 8002.
    Retorna {ok, image_base64, source: "local"|"nexus", evidence_path?, error?}.
    """
    # 0) Visión Ubicua (ojo activo): si está seleccionado, priorizarlo.
    # Formato: "ubiq:<cam_id>" o setting persistente `vision.active_eye`.
    try:
        requested = (eye or "").strip()
        if not requested:
            from modules.humanoid.vision.ubiq.registry import get_setting

            requested = (get_setting("vision.active_eye") or "").strip()
        if requested.lower().startswith("ubiq:"):
            cam_id = requested.split(":", 1)[1].strip()
            if cam_id:
                from modules.humanoid.vision.ubiq.streaming import take_snapshot

                snap = take_snapshot(cam_id, timeout_s=3.5)
                if snap.get("ok") and snap.get("image_base64"):
                    return {
                        "ok": True,
                        "image_base64": snap["image_base64"],
                        "source": "ubiq",
                        "evidence_path": None,
                        "error": None,
                        "eye": requested,
                        "cam_id": cam_id,
                    }
    except Exception:
        pass

    if use_nexus_if_available and _nexus_eyes_available():
        raw = _nexus_snapshot_advanced(
            source=source,
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
            index=index,
        )
        if raw is None:
            # fallback simple
            raw = _nexus_snapshot(source=source)
        if raw:
            b64 = base64.b64encode(raw).decode("ascii")
            return {
                "ok": True,
                "image_base64": b64,
                "source": "nexus",
                "evidence_path": None,
                "error": None,
            }
    try:
        from modules.humanoid.hands_eyes.engine import capture_full_scene
        out = capture_full_scene(region=region, save_evidence=True)
        if not out.get("ok"):
            return {
                "ok": False,
                "image_base64": None,
                "source": "local",
                "evidence_path": None,
                "error": out.get("error", "capture_failed"),
            }
        b64 = None
        if out.get("base64"):
            b64 = out["base64"]
        elif out.get("png_bytes"):
            b64 = base64.b64encode(out["png_bytes"]).decode("ascii")
        return {
            "ok": True,
            "image_base64": b64,
            "source": "local",
            "evidence_path": out.get("evidence_path"),
            "error": None,
        }
    except Exception as e:
        return {
            "ok": False,
            "image_base64": None,
            "source": "local",
            "evidence_path": None,
            "error": str(e),
        }


def nerve_eyes_status() -> Dict[str, Any]:
    """Estado del nervio ojos: Nexus disponible, URL snapshot."""
    nexus_ok = _nexus_eyes_available()
    return {
        "nexus_available": nexus_ok,
        "nexus_snapshot_url": f"{NEXUS_ROBOT_API_URL}/api/vision/snapshot" if NEXUS_ENABLED else None,
        "nexus_snapshot_supports": ["source", "enhance", "jpeg_quality", "focus_x", "focus_y", "zoom", "index"],
        "local_fallback": True,
    }
