"""Tailscale wrapper: status, ping, resolve node to URL."""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional

TIMEOUT = 10


def _tailscale_path() -> Optional[Path]:
    p = os.getenv("TAILSCALE_EXE_PATH", "C:\\Program Files\\Tailscale\\tailscale.exe")
    path = Path(p).expanduser().resolve()
    if path.exists():
        return path
    try:
        r = subprocess.run(["where", "tailscale"], capture_output=True, text=True, timeout=5)
        if r.returncode == 0 and r.stdout:
            for line in r.stdout.strip().splitlines():
                line = line.strip()
                if line.lower().endswith("tailscale.exe"):
                    return Path(line)
    except Exception:
        pass
    return None


def status() -> Dict[str, Any]:
    """Tailscale status (parse output)."""
    path = _tailscale_path()
    if not path:
        return {"ok": False, "available": False, "error": "missing_deps"}
    try:
        r = subprocess.run([str(path), "status", "--json"], capture_output=True, text=True, timeout=TIMEOUT)
        if r.returncode != 0:
            return {"ok": False, "available": True, "error": r.stderr or "status failed"}
        import json
        data = json.loads(r.stdout) if r.stdout else {}
        return {"ok": True, "available": True, "data": data}
    except Exception as e:
        return {"ok": False, "available": True, "error": str(e)}


def ping(node: str) -> Dict[str, Any]:
    """Tailscale ping node. Returns {ok, latency?}."""
    path = _tailscale_path()
    if not path:
        return {"ok": False, "error": "missing_deps"}
    try:
        r = subprocess.run([str(path), "ping", "-c", "1", node], capture_output=True, text=True, timeout=TIMEOUT)
        ok = r.returncode == 0
        latency = None
        if r.stdout:
            m = re.search(r"(\d+\.?\d*)\s*ms", r.stdout)
            if m:
                latency = float(m.group(1))
        return {"ok": ok, "latency_ms": latency, "error": None if ok else (r.stderr or r.stdout or "ping failed")}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def get_worker_url() -> Optional[str]:
    """Resolve worker URL: http://<TAILSCALE_DNS_NAME or expected_node>:8791."""
    dns = os.getenv("TAILSCALE_DNS_NAME", "").strip() or os.getenv("TAILSCALE_EXPECTED_NODE", "").strip()
    if not dns:
        return None
    return f"http://{dns}:8791"
