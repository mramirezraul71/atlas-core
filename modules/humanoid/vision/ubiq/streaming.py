from __future__ import annotations

import base64
import os
import secrets
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from .registry import get_camera, get_setting, set_setting


_LOCK = threading.Lock()
_PROCS: Dict[str, subprocess.Popen] = {}  # key -> popen


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _streams_root() -> Path:
    # bajo snapshots para que `api_evidence_image`/seguridad sea consistente
    return _repo_root() / "snapshots" / "vision" / "ubiq_streams"


def _ffmpeg() -> str:
    return (os.getenv("FFMPEG_PATH") or "ffmpeg").strip() or "ffmpeg"


def _token_file() -> Path:
    return _repo_root() / "logs" / "mobile_stream_token.txt"


def get_mobile_token() -> str:
    # 1) env explícito
    env_tok = (os.getenv("ATLAS_MOBILE_STREAM_TOKEN") or "").strip()
    if env_tok:
        return env_tok
    # 2) settings persistente
    tok = (get_setting("vision.mobile_token") or "").strip()
    if tok:
        return tok
    # 3) file persistente
    p = _token_file()
    try:
        if p.is_file():
            t = (p.read_text(encoding="utf-8", errors="ignore") or "").strip()
            if t:
                set_setting("vision.mobile_token", t)
                return t
    except Exception:
        pass
    # 4) generar
    tok = secrets.token_urlsafe(24)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(tok, encoding="utf-8")
    except Exception:
        pass
    set_setting("vision.mobile_token", tok)
    return tok


def get_stream_paths(cam_id: str) -> Dict[str, str]:
    cid = (cam_id or "").strip()
    base = (_streams_root() / cid).resolve()
    local_dir = (base / "local").resolve()
    mobile_dir = (base / "mobile").resolve()
    return {
        "base_dir": str(base),
        "local_dir": str(local_dir),
        "mobile_dir": str(mobile_dir),
        "local_playlist": str(local_dir / "index.m3u8"),
        "mobile_playlist": str(mobile_dir / "index.m3u8"),
        "mobile_key": str(mobile_dir / "enc.key"),
        "mobile_keyinfo": str(mobile_dir / "enc.keyinfo"),
    }


def _build_ffmpeg_cmd(url: str, out_dir: Path, *, encrypted: bool, key_uri: str = "") -> Tuple[list, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    playlist = out_dir / "index.m3u8"
    seg_pat = str(out_dir / "seg%05d.ts")

    input_args = []
    if url.lower().startswith("rtsp://"):
        input_args += ["-rtsp_transport", "tcp"]
    input_args += ["-i", url]

    hls_args = [
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-tune",
        "zerolatency",
        "-g",
        "50",
        "-sc_threshold",
        "0",
        "-f",
        "hls",
        "-hls_time",
        "2",
        "-hls_list_size",
        "6",
        "-hls_flags",
        "delete_segments+append_list+independent_segments",
        "-hls_segment_filename",
        seg_pat,
    ]

    if encrypted:
        # AES-128 HLS: keyinfo: 3 lines -> URI, keyfile path, IV(optional)
        key_file = out_dir / "enc.key"
        keyinfo = out_dir / "enc.keyinfo"
        if not key_file.exists():
            key_file.write_bytes(os.urandom(16))
        if not key_uri:
            key_uri = "/api/vision/ubiq/streams/key"
        keyinfo.write_text(f"{key_uri}\n{str(key_file)}\n", encoding="utf-8")
        hls_args += ["-hls_key_info_file", str(keyinfo)]

    cmd = [_ffmpeg(), "-hide_banner", "-loglevel", "error"] + input_args + hls_args + [str(playlist)]
    return cmd, playlist


def start_stream(cam_id: str, variant: str = "local") -> Dict[str, Any]:
    """
    Lanza FFmpeg para una cámara hacia HLS.
    variant=local (sin cifrar) | mobile (cifrado AES-128)
    """
    cid = (cam_id or "").strip()
    if not cid:
        return {"ok": False, "error": "missing cam_id"}
    cam = get_camera(cid)
    if not cam:
        return {"ok": False, "error": "camera_not_found", "cam_id": cid}
    url = str(cam.get("url") or "").strip()
    if not url:
        return {"ok": False, "error": "camera_missing_url", "cam_id": cid}

    paths = get_stream_paths(cid)
    out_dir = Path(paths["local_dir"] if variant == "local" else paths["mobile_dir"])
    encrypted = variant == "mobile"

    key_uri = ""
    if encrypted:
        tok = get_mobile_token()
        # Token en query: compatible con players móviles HLS (sin headers custom)
        key_uri = f"/api/vision/ubiq/streams/{cid}/key?token={tok}"

    proc_key = f"{cid}:{variant}"
    with _LOCK:
        p = _PROCS.get(proc_key)
        if p and p.poll() is None:
            return {"ok": True, "already_running": True, "cam_id": cid, "variant": variant, "paths": paths}

        cmd, playlist = _build_ffmpeg_cmd(url, out_dir, encrypted=encrypted, key_uri=key_uri)
        try:
            # Windows: CREATE_NO_WINDOW para no abrir consola
            creationflags = 0x08000000 if os.name == "nt" else 0
            p = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                creationflags=creationflags,
            )
            _PROCS[proc_key] = p
        except Exception as e:
            return {"ok": False, "error": str(e), "cam_id": cid, "variant": variant, "cmd": cmd}

    return {"ok": True, "cam_id": cid, "variant": variant, "paths": paths}


def stop_stream(cam_id: str, variant: str = "local") -> Dict[str, Any]:
    cid = (cam_id or "").strip()
    proc_key = f"{cid}:{variant}"
    with _LOCK:
        p = _PROCS.get(proc_key)
        if not p:
            return {"ok": True, "stopped": False, "reason": "not_running"}
        try:
            p.terminate()
        except Exception:
            pass
        try:
            p.wait(timeout=2)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
        _PROCS.pop(proc_key, None)
    return {"ok": True, "stopped": True, "cam_id": cid, "variant": variant}


def stream_status() -> Dict[str, Any]:
    with _LOCK:
        items = list(_PROCS.items())
    out = []
    for k, p in items:
        out.append({"key": k, "running": p.poll() is None, "pid": getattr(p, "pid", None)})
    return {"ok": True, "processes": out, "count": len(out), "ffmpeg": _ffmpeg()}


def take_snapshot(cam_id: str, timeout_s: float = 3.5) -> Dict[str, Any]:
    """
    Captura un frame JPEG (bytes + base64) desde una cámara UBIQ.
    Usa FFmpeg directo (no requiere OpenCV).
    """
    cid = (cam_id or "").strip()
    cam = get_camera(cid)
    if not cam:
        return {"ok": False, "error": "camera_not_found", "cam_id": cid}
    url = str(cam.get("url") or "").strip()
    if not url:
        return {"ok": False, "error": "camera_missing_url", "cam_id": cid}

    cmd = [_ffmpeg(), "-hide_banner", "-loglevel", "error"]
    if url.lower().startswith("rtsp://"):
        cmd += ["-rtsp_transport", "tcp"]
    cmd += ["-i", url, "-frames:v", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=float(timeout_s))
        if r.returncode != 0 or not r.stdout:
            return {"ok": False, "error": "snapshot_failed", "stderr": (r.stderr or b"")[:400].decode("utf-8", "ignore")}
        b = r.stdout
        return {"ok": True, "jpeg_bytes": b, "image_base64": base64.b64encode(b).decode("ascii")}
    except Exception as e:
        return {"ok": False, "error": str(e), "cmd": cmd}

