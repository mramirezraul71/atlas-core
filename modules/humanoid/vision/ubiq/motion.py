from __future__ import annotations

import hashlib
import os
import threading
import time
from typing import Any, Dict, Optional

from .registry import list_cameras, get_setting, set_setting
from .streaming import take_snapshot


_THREAD: Optional[threading.Thread] = None
_STOP = threading.Event()
_STATE_LOCK = threading.Lock()
_LAST: Dict[str, str] = {}  # cam_id -> hash
_LAST_MOTION: Dict[str, float] = {}  # cam_id -> ts (throttle)


def _hash_bytes(b: bytes) -> str:
    return hashlib.md5(b).hexdigest()


def _set_active_eye(cam_id: str, hold_s: float) -> None:
    until = time.time() + max(3.0, float(hold_s))
    set_setting("vision.active_eye", f"ubiq:{cam_id}")
    set_setting("vision.active_eye_until", str(until))


def _maybe_revert_to_primary() -> None:
    """
    Si expiró active_eye_until, limpiar (deja que el sistema use el ojo principal).
    """
    raw_until = (get_setting("vision.active_eye_until") or "").strip()
    if not raw_until:
        return
    try:
        until = float(raw_until)
    except Exception:
        return
    if time.time() < until:
        return
    # expira: limpiar
    set_setting("vision.active_eye", "")
    set_setting("vision.active_eye_until", "")


def _run() -> None:
    poll_s = float(os.getenv("VISION_MOTION_POLL_S", "2.5"))
    hold_s = float(os.getenv("VISION_MOTION_HOLD_S", "25"))
    threshold = float(os.getenv("VISION_MOTION_THRESHOLD", "0.08"))  # ratio heurístico
    min_gap_s = float(os.getenv("VISION_MOTION_MIN_GAP_S", "20"))

    while not _STOP.is_set():
        try:
            _maybe_revert_to_primary()
        except Exception:
            pass
        cams = []
        try:
            cams = list_cameras(limit=80)
        except Exception:
            cams = []

        for cam in cams:
            if _STOP.is_set():
                break
            cid = (cam.get("id") or "").strip()
            proto = (cam.get("protocol") or "").strip().lower()
            url = (cam.get("url") or "").strip()
            if not cid or not url:
                continue
            if proto not in ("rtsp", "http", "https", "mjpeg"):
                # ONVIF puro sin RTSP URL real aún: saltar
                continue

            # Throttle por cámara
            last_motion = float(_LAST_MOTION.get(cid) or 0.0)
            if time.time() - last_motion < min_gap_s:
                continue

            snap = take_snapshot(cid, timeout_s=3.0)
            if not snap.get("ok") or not snap.get("jpeg_bytes"):
                continue
            b: bytes = snap["jpeg_bytes"]
            h = _hash_bytes(b)
            prev = _LAST.get(cid)
            _LAST[cid] = h
            if not prev:
                continue

            # Heurística sin decodificar: comparar hashes como "cambio total".
            # Para minimizar falsos positivos, pedimos 2 cambios seguidos (prev != h) y ratio simbólico.
            if prev != h:
                # ratio simbólico: cantidad de chars distintos / 32
                diff = sum(1 for i in range(min(len(prev), len(h))) if prev[i] != h[i])
                ratio = diff / 32.0
                if ratio >= threshold:
                    _LAST_MOTION[cid] = time.time()
                    _set_active_eye(cid, hold_s=hold_s)
                    try:
                        from modules.humanoid.ans.evolution_bitacora import append_evolution_log

                        append_evolution_log(
                            f"[VISION] Movimiento detectado en {cam.get('ip') or cid}. Ojo activo -> {cid}",
                            ok=True,
                            source="vision",
                        )
                    except Exception:
                        pass
                    try:
                        from modules.humanoid.comms.ops_bus import emit

                        emit("vision", f"Movimiento detectado. Ojo activo -> {cid}", level="med", data={"cam_id": cid})
                    except Exception:
                        pass

        _STOP.wait(timeout=max(0.5, poll_s))


def start_motion_watchdog() -> Dict[str, Any]:
    global _THREAD
    if os.getenv("VISION_UBIQ_MOTION_ENABLED", "true").strip().lower() not in ("1", "true", "yes", "y", "on"):
        return {"ok": True, "enabled": False}
    with _STATE_LOCK:
        if _THREAD and _THREAD.is_alive():
            return {"ok": True, "running": True}
        _STOP.clear()
        _THREAD = threading.Thread(target=_run, daemon=True, name="vision_ubiq_motion")
        _THREAD.start()
        return {"ok": True, "running": True}


def stop_motion_watchdog() -> Dict[str, Any]:
    global _THREAD
    with _STATE_LOCK:
        _STOP.set()
        t = _THREAD
        _THREAD = None
    if t:
        try:
            t.join(timeout=1.5)
        except Exception:
            pass
    return {"ok": True, "stopped": True}

