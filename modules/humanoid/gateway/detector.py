"""Detect installed gateway tools and candidates."""
from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

TIMEOUT_SEC = 5


def _env_path(name: str, default: str) -> Path:
    p = os.getenv(name, default)
    return Path(p).expanduser().resolve() if p else Path()


def _exe_exists(path: Path) -> bool:
    if not path:
        return False
    return path.exists() and path.suffix.lower() in (".exe", "") and path.is_file()


def resolve_cloudflared_path() -> Optional[Path]:
    """Localiza ``cloudflared.exe``: variable de entorno, bin ATLAS, instalación típica, ``where``."""
    raw = (os.getenv("CLOUDFLARE_CLOUDFLARED_PATH") or "").strip()
    push_root = Path(os.getenv("ATLAS_PUSH_ROOT", r"C:\ATLAS_PUSH")).expanduser().resolve()
    candidates: list[Path] = []
    if raw:
        p = Path(raw).expanduser()
        candidates.append(p.resolve() if p.is_absolute() else (Path(os.getcwd()) / p).resolve())
    candidates.append((push_root / "bin" / "cloudflared.exe").resolve())
    candidates.append(Path(r"C:\Program Files (x86)\cloudflared\cloudflared.exe"))
    candidates.append(Path(r"C:\Program Files\cloudflared\cloudflared.exe"))
    for path in candidates:
        if _exe_exists(path):
            return path
    try:
        r = subprocess.run(
            ["where", "cloudflared"],
            capture_output=True,
            text=True,
            timeout=TIMEOUT_SEC,
        )
        if r.returncode == 0 and (r.stdout or "").strip():
            w = Path((r.stdout or "").strip().splitlines()[0].strip())
            if _exe_exists(w):
                return w
    except Exception:
        pass
    return None


def detect_cloudflared() -> Dict[str, Any]:
    path = resolve_cloudflared_path() or _env_path(
        "CLOUDFLARE_CLOUDFLARED_PATH", "C:\\ATLAS_PUSH\\bin\\cloudflared.exe"
    )
    if not path.is_absolute():
        path = Path(os.getcwd()) / path
    present = _exe_exists(path) if path else False
    token = bool(os.getenv("CLOUDFLARE_TOKEN", "").strip())
    tunnel_url = os.getenv("CLOUDFLARE_TUNNEL_URL", "").strip()
    tunnel_name = os.getenv("CLOUDFLARE_TUNNEL_NAME", "").strip()
    enabled = os.getenv("CLOUDFLARE_TUNNEL_ENABLED", "").strip().lower() in ("1", "true", "yes")
    return {
        "available": present,
        "path": str(path),
        "enabled": enabled,
        "has_token": token,
        "has_tunnel_url": bool(tunnel_url),
        "has_tunnel_name": bool(tunnel_name),
        "missing_deps": not present,
    }


def detect_tailscale() -> Dict[str, Any]:
    path = _env_path("TAILSCALE_EXE_PATH", "C:\\Program Files\\Tailscale\\tailscale.exe")
    present = _exe_exists(path)
    if not present:
        try:
            r = subprocess.run(["where", "tailscale"], capture_output=True, text=True, timeout=TIMEOUT_SEC)
            present = r.returncode == 0 and "tailscale" in (r.stdout or "").lower()
        except Exception:
            pass
    enabled = os.getenv("TAILSCALE_ENABLED", "").strip().lower() in ("1", "true", "yes")
    node = os.getenv("TAILSCALE_EXPECTED_NODE", "").strip()
    dns = os.getenv("TAILSCALE_DNS_NAME", "").strip()
    return {
        "available": present,
        "path": str(path) if path else "",
        "enabled": enabled,
        "expected_node": node,
        "dns_name": dns,
        "missing_deps": not present,
    }


def detect_ssh() -> Dict[str, Any]:
    path = _env_path("SSH_EXE_PATH", "C:\\Windows\\System32\\OpenSSH\\ssh.exe")
    present = _exe_exists(path)
    if not present:
        try:
            r = subprocess.run(["where", "ssh"], capture_output=True, text=True, timeout=TIMEOUT_SEC)
            present = r.returncode == 0 and "ssh" in (r.stdout or "").lower()
        except Exception:
            pass
    enabled = os.getenv("SSH_TUNNEL_ENABLED", "").strip().lower() in ("1", "true", "yes")
    user = os.getenv("SSH_USER", "").strip()
    host = os.getenv("SSH_HOST", "").strip()
    return {
        "available": present,
        "path": str(path) if path else "",
        "enabled": enabled,
        "has_user_host": bool(user and host),
        "config_missing": not (user and host),
        "missing_deps": not present,
    }


def detect_lan() -> Dict[str, Any]:
    url = os.getenv("LAN_WORKER_URL", "").strip()
    enabled = os.getenv("LAN_ENABLED", "").strip().lower() in ("1", "true", "yes")
    return {
        "available": bool(url),
        "url": url,
        "enabled": enabled,
    }


def detect_all() -> Dict[str, Any]:
    return {
        "cloudflare": detect_cloudflared(),
        "tailscale": detect_tailscale(),
        "ssh": detect_ssh(),
        "lan": detect_lan(),
    }
