"""SSH reverse tunnel wrapper: start -R remote:local, status."""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

TIMEOUT = 5


def _ssh_path() -> Optional[Path]:
    p = os.getenv("SSH_EXE_PATH", "C:\\Windows\\System32\\OpenSSH\\ssh.exe")
    path = Path(p)
    if path.exists():
        return path
    try:
        r = subprocess.run(["where", "ssh"], capture_output=True, text=True, timeout=5)
        if r.returncode == 0 and r.stdout:
            return Path(r.stdout.strip().splitlines()[0].strip())
    except Exception:
        pass
    return None


def status() -> Dict[str, Any]:
    """Check SSH and config."""
    path = _ssh_path()
    if not path:
        return {"ok": False, "available": False, "error": "missing_deps"}
    user = os.getenv("SSH_USER", "").strip()
    host = os.getenv("SSH_HOST", "").strip()
    if not user or not host:
        return {"ok": False, "available": True, "error": "config_missing", "message": "set SSH_USER and SSH_HOST"}
    return {"ok": True, "available": True, "user": user, "host": host}


def get_worker_url() -> Optional[str]:
    """HQ reaches worker via SSH reverse: http://SSH_HOST:SSH_REMOTE_PORT (HQ must listen there)."""
    host = os.getenv("SSH_HOST", "").strip()
    if not host:
        return None
    port = os.getenv("SSH_REMOTE_PORT", "18791").strip() or "18791"
    return f"http://{host}:{port}"


def start() -> Dict[str, Any]:
    """Start SSH reverse tunnel: -N -R remote_port:127.0.0.1:local_port user@host."""
    path = _ssh_path()
    if not path:
        return {"ok": False, "error": "missing_deps"}
    user = os.getenv("SSH_USER", "").strip()
    host = os.getenv("SSH_HOST", "").strip()
    remote_port = os.getenv("SSH_REMOTE_PORT", "18791").strip() or "18791"
    local_port = os.getenv("SSH_LOCAL_PORT", "8791").strip() or "8791"
    if not user or not host:
        return {"ok": False, "error": "config_missing"}
    try:
        proc = subprocess.Popen(
            [str(path), "-N", "-R", f"{remote_port}:127.0.0.1:{local_port}", f"{user}@{host}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        time.sleep(1)
        if proc.poll() is not None:
            err = (proc.stderr.read() or b"").decode()[:500]
            return {"ok": False, "error": err or "ssh exited"}
        return {"ok": True, "pid": proc.pid}
    except Exception as e:
        return {"ok": False, "error": str(e)}
