"""Cloudflare Tunnel wrapper: start/stop/status, parse logs for URL."""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

TIMEOUT_START = 15


def _cloudflared_path() -> Path:
    p = os.getenv("CLOUDFLARE_CLOUDFLARED_PATH", "C:\\ATLAS_PUSH\\bin\\cloudflared.exe")
    return Path(p).expanduser().resolve()


def status() -> Dict[str, Any]:
    """Check if cloudflared exists and config present."""
    path = _cloudflared_path()
    if not path.exists():
        return {"ok": False, "available": False, "error": "missing_deps", "message": "cloudflared.exe not found"}
    token = os.getenv("CLOUDFLARE_TOKEN", "").strip()
    tunnel_url = os.getenv("CLOUDFLARE_TUNNEL_URL", "").strip()
    tunnel_name = os.getenv("CLOUDFLARE_TUNNEL_NAME", "").strip()
    if token:
        return {"ok": True, "available": True, "mode": "token", "message": "token configured"}
    if tunnel_url:
        return {"ok": True, "available": True, "mode": "named", "url": tunnel_url, "message": "tunnel URL set"}
    if tunnel_name:
        return {"ok": True, "available": True, "mode": "named", "tunnel_name": tunnel_name, "message": "tunnel name set"}
    return {"ok": False, "available": True, "error": "config_missing", "message": "set CLOUDFLARE_TOKEN or CLOUDFLARE_TUNNEL_URL"}


def get_worker_url() -> Optional[str]:
    """Return worker URL: from CLOUDFLARE_TUNNEL_URL or https://<tunnel_name>."""
    url = os.getenv("CLOUDFLARE_TUNNEL_URL", "").strip()
    if url:
        return url.rstrip("/")
    name = os.getenv("CLOUDFLARE_TUNNEL_NAME", "").strip()
    if name:
        return f"https://{name}"
    return None


def start() -> Dict[str, Any]:
    """Start tunnel: token mode or named. Does not block; returns {ok, pid?, error}."""
    path = _cloudflared_path()
    if not path.exists():
        return {"ok": False, "error": "missing_deps"}
    token = os.getenv("CLOUDFLARE_TOKEN", "").strip()
    if token:
        try:
            proc = subprocess.Popen(
                [str(path), "tunnel", "run", "--token", token],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                cwd=str(path.parent),
            )
            time.sleep(2)
            if proc.poll() is not None:
                err = (proc.stderr.read() or b"").decode()[:500]
                return {"ok": False, "error": err or "tunnel exited"}
            return {"ok": True, "pid": proc.pid}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    tunnel_name = os.getenv("CLOUDFLARE_TUNNEL_NAME", "").strip()
    if tunnel_name:
        try:
            proc = subprocess.Popen(
                [str(path), "tunnel", "run", tunnel_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                cwd=str(path.parent),
            )
            time.sleep(2)
            if proc.poll() is not None:
                err = (proc.stderr.read() or b"").decode()[:500]
                return {"ok": False, "error": err or "tunnel exited"}
            return {"ok": True, "pid": proc.pid}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    return {"ok": False, "error": "config_missing"}
