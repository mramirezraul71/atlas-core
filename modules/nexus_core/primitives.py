from __future__ import annotations

import base64
import os
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _clamp(v: float, lo: float, hi: float) -> float:
    try:
        x = float(v)
    except Exception:
        x = 0.0
    return max(lo, min(hi, x))


def navigate_to(x: int, y: int, *, duration: float = 0.2, expected_window: str = "", expected_process: str = "") -> Dict[str, Any]:
    """
    Posiciona el cursor en coordenadas de pantalla (x,y).
    Wrapper seguro sobre `modules.humanoid.screen.actions.execute_action('move', ...)`.
    """
    try:
        from modules.humanoid.screen.actions import execute_action

        payload = {"x": int(x), "y": int(y), "duration": float(duration or 0.0)}
        if expected_window:
            payload["expected_window_title"] = expected_window
        if expected_process:
            payload["expected_process"] = expected_process
        r = execute_action("move", payload)
        return {"ok": bool(r.get("ok")), "result": r, "error": r.get("error")}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}


def reach_pose(pan: float, tilt: float, zoom: float = 1.0, *, source: str = "camera") -> Dict[str, Any]:
    """
    Control de orientación (best-effort).
    Nota: en el stack actual se implementa como "PTZ digital" (focus_x/focus_y/zoom) en NEXUS Robot.
    pan/tilt en [-1..1] se mapean a focus_x/focus_y en [0..1].
    """
    focus_x = _clamp(0.5 + (_clamp(pan, -1.0, 1.0) * 0.45), 0.0, 1.0)
    focus_y = _clamp(0.5 + (_clamp(tilt, -1.0, 1.0) * 0.45), 0.0, 1.0)
    z = _clamp(zoom, 1.0, 4.0)
    base = (os.getenv("NEXUS_ROBOT_API_URL") or os.getenv("ROBOT_BASE_URL") or "http://127.0.0.1:8002").rstrip("/")
    url = f"{base}/api/vision/snapshot?source={source}&focus_x={focus_x:.3f}&focus_y={focus_y:.3f}&zoom={z:.3f}&enhance=max"
    t0 = time.perf_counter()
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=6) as r:
            ok = r.status == 200
            data = r.read() if ok else b""
        ms = int((time.perf_counter() - t0) * 1000)
        # Persistir "pose" actual (útil para reproducibilidad)
        try:
            pose_path = _repo_root() / "logs" / "nexus_pose.json"
            pose_path.parent.mkdir(parents=True, exist_ok=True)
            pose_path.write_text(
                f'{{"pan":{float(pan):.3f},"tilt":{float(tilt):.3f},"zoom":{float(z):.3f},"focus_x":{focus_x:.3f},"focus_y":{focus_y:.3f},"ts":{time.time():.3f}}}',
                encoding="utf-8",
            )
        except Exception:
            pass
        return {
            "ok": bool(ok),
            "ms": ms,
            "pose": {"pan": float(pan), "tilt": float(tilt), "zoom": float(z), "focus_x": focus_x, "focus_y": focus_y},
            "snapshot_jpeg_b64": base64.b64encode(data).decode("ascii") if data else "",
            "error": None if ok else "snapshot_failed",
            "url": url,
        }
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "ms": ms, "pose": {"pan": pan, "tilt": tilt, "zoom": z, "focus_x": focus_x, "focus_y": focus_y}, "error": str(e)[:200], "url": url}


def grasp(target_id: str = "screen", *, region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    """
    Captura de región/ventana/frame para análisis.
    target_id:
      - "screen" (default)
      - "ubiq:<cam_id>"
      - "nexus:camera" / "nexus:screen"
    """
    tid = (target_id or "screen").strip()
    out_dir = _repo_root() / "snapshots" / "grasp"
    out_dir.mkdir(parents=True, exist_ok=True)

    if tid.lower().startswith("ubiq:"):
        cam_id = tid.split(":", 1)[1].strip()
        try:
            from modules.humanoid.vision.ubiq.streaming import take_snapshot
            snap = take_snapshot(cam_id, timeout_s=4.0)
            if not snap.get("ok") or not snap.get("jpeg_bytes"):
                return {"ok": False, "error": snap.get("error") or "snapshot_failed", "target": tid}
            path = out_dir / f"ubiq_{cam_id}_{int(time.time())}.jpg"
            path.write_bytes(snap["jpeg_bytes"])
            return {"ok": True, "resource_id": f"file:{path}", "path": str(path), "image_base64": snap.get("image_base64"), "target": tid}
        except Exception as e:
            return {"ok": False, "error": str(e)[:200], "target": tid}

    if tid.lower().startswith("nexus:"):
        src = tid.split(":", 1)[1].strip() or "camera"
        try:
            from modules.humanoid.nerve.eyes import eyes_capture
            r = eyes_capture(use_nexus_if_available=True, source=src)
            if not r.get("ok") or not r.get("image_base64"):
                return {"ok": False, "error": r.get("error") or "nexus_capture_failed", "target": tid}
            b = base64.b64decode(r["image_base64"])
            path = out_dir / f"nexus_{src}_{int(time.time())}.jpg"
            path.write_bytes(b)
            return {"ok": True, "resource_id": f"file:{path}", "path": str(path), "image_base64": r.get("image_base64"), "target": tid}
        except Exception as e:
            return {"ok": False, "error": str(e)[:200], "target": tid}

    # screen (default)
    try:
        from modules.humanoid.screen.capture import capture_screen, save_capture_to_file
        png, err = capture_screen(region=region)
        if err or not png:
            return {"ok": False, "error": err or "capture_failed", "target": tid}
        path = Path(save_capture_to_file(png, str(out_dir), prefix="screen"))
        return {"ok": True, "resource_id": f"file:{path}", "path": str(path), "image_base64": base64.b64encode(png).decode("ascii"), "target": tid}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "target": tid}


def release(resource_id: str) -> Dict[str, Any]:
    """
    Libera un recurso (best-effort).
    Formatos soportados:
      - stream:<cam_id>:<variant>  -> stop_stream
      - proc:<pid>                -> taskkill
      - file:<path>               -> no borra por defecto (solo valida existencia)
    """
    rid = (resource_id or "").strip()
    if not rid:
        return {"ok": True, "released": False, "reason": "empty"}
    if rid.startswith("stream:"):
        try:
            _, rest = rid.split(":", 1)
            cam_id, variant = rest.split(":", 1)
            from modules.humanoid.vision.ubiq.streaming import stop_stream
            r = stop_stream(cam_id.strip(), variant.strip())
            return {"ok": bool(r.get("ok")), "released": bool(r.get("stopped")), "result": r}
        except Exception as e:
            return {"ok": False, "released": False, "error": str(e)[:200]}
    if rid.startswith("proc:"):
        try:
            pid = int(rid.split(":", 1)[1])
            import subprocess
            subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True, text=True, timeout=10)
            return {"ok": True, "released": True, "pid": pid}
        except Exception as e:
            return {"ok": False, "released": False, "error": str(e)[:200]}
    if rid.startswith("file:"):
        p = Path(rid.split(":", 1)[1])
        exists = p.exists()
        # No borrar evidencia por defecto.
        return {"ok": True, "released": True, "exists": exists, "path": str(p)}
    return {"ok": True, "released": False, "reason": "unknown_resource_id"}


def pulse_check() -> Dict[str, Any]:
    """
    Diagnóstico de latencia y estado entre PUSH (8791) y NEXUS (8000).
    Devuelve latencias y estado.
    """
    push = (os.getenv("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791").rstrip("/")
    nexus = (os.getenv("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/")
    out: Dict[str, Any] = {"ok": True, "push": {}, "nexus": {}}

    def _ping(url: str) -> Dict[str, Any]:
        t0 = time.perf_counter()
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=4) as r:
                status = r.status
                _ = r.read(256)
            ms = int((time.perf_counter() - t0) * 1000)
            return {"ok": status == 200, "status": status, "ms": ms}
        except Exception as e:
            ms = int((time.perf_counter() - t0) * 1000)
            return {"ok": False, "status": None, "ms": ms, "error": str(e)[:160]}

    out["push"] = _ping(push + "/health")
    # NEXUS: prefer ping_nexus (health/status fallback)
    try:
        from modules.nexus_heartbeat import ping_nexus
        t0 = time.perf_counter()
        ok, msg = ping_nexus()
        out["nexus"] = {"ok": bool(ok), "ms": int((time.perf_counter() - t0) * 1000), "message": msg}
    except Exception:
        out["nexus"] = _ping(nexus + "/health")

    out["ok"] = bool(out["push"].get("ok") and out["nexus"].get("ok"))
    return out

