"""Auto mode: choose best gateway path. Order: cloudflare -> tailscale -> ssh -> lan."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from . import cloudflare
from . import tailscale
from . import ssh_tunnel
from . import lan
from . import detector
from . import store
from .models import WorkerTarget


def _order() -> List[str]:
    return ["cloudflare", "tailscale", "ssh", "lan"]


def resolve_worker_url(mode: Optional[str] = None) -> Optional[WorkerTarget]:
    """Resolve worker base_url for given mode (or store mode)."""
    m = (mode or store.get_mode()).strip().lower()
    if m == "auto":
        for name in _order():
            target = _resolve_one(name)
            if target:
                return target
        return None
    return _resolve_one(m)


def _resolve_one(name: str) -> Optional[WorkerTarget]:
    if name == "cloudflare":
        url = cloudflare.get_worker_url()
        if url:
            return WorkerTarget(mode="cloudflare", base_url=url)
    elif name == "tailscale":
        url = tailscale.get_worker_url()
        if url:
            return WorkerTarget(mode="tailscale", base_url=url)
    elif name == "ssh":
        url = ssh_tunnel.get_worker_url()
        if url:
            return WorkerTarget(mode="ssh", base_url=url)
    elif name == "lan":
        url = lan.get_worker_url()
        if url:
            return WorkerTarget(mode="lan", base_url=url)
    return None


def check_candidates() -> Dict[str, Any]:
    """Return which modes are available (for UI)."""
    tools = detector.detect_all()
    result = {}
    for name in _order():
        d = tools.get(name, {})
        available = d.get("available") and d.get("enabled", True)
        url = None
        if name == "cloudflare":
            url = cloudflare.get_worker_url()
        elif name == "tailscale":
            url = tailscale.get_worker_url()
        elif name == "ssh":
            url = ssh_tunnel.get_worker_url()
        elif name == "lan":
            url = lan.get_worker_url()
        result[name] = {"available": available, "url": url, "detail": d}
    return result
