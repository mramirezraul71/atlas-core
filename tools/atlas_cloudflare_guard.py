from __future__ import annotations

import ipaddress
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Tuple


BASE_DIR = Path(__file__).resolve().parent.parent
CF_DIR = BASE_DIR / "config" / "cloudflare"
V4_FILE = CF_DIR / "ips-v4.txt"
V6_FILE = CF_DIR / "ips-v6.txt"


def refresh_cloudflare_ranges(timeout: int = 8) -> Tuple[bool, str]:
    CF_DIR.mkdir(parents=True, exist_ok=True)
    try:
        req4 = urllib.request.Request(
            "https://www.cloudflare.com/ips-v4",
            headers={"User-Agent": "ATLAS-Cloudflare-Guard/1.0"},
        )
        req6 = urllib.request.Request(
            "https://www.cloudflare.com/ips-v6",
            headers={"User-Agent": "ATLAS-Cloudflare-Guard/1.0"},
        )
        with urllib.request.urlopen(req4, timeout=timeout) as r:
            V4_FILE.write_text(r.read().decode("utf-8", errors="replace"), encoding="utf-8")
        with urllib.request.urlopen(req6, timeout=timeout) as r:
            V6_FILE.write_text(r.read().decode("utf-8", errors="replace"), encoding="utf-8")
        return True, "ok"
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def _load_ranges() -> List[Any]:
    nets: List[Any] = []
    for p in [V4_FILE, V6_FILE]:
        if not p.exists():
            continue
        for ln in p.read_text(encoding="utf-8", errors="replace").splitlines():
            line = ln.strip()
            if not line:
                continue
            try:
                nets.append(ipaddress.ip_network(line, strict=False))
            except Exception:
                continue
    return nets


def is_cloudflare_edge_ip(ip: str) -> bool:
    try:
        addr = ipaddress.ip_address(ip)
    except Exception:
        return False
    for net in _load_ranges():
        if addr in net:
            return True
    return False


def authorize_dashboard_request(remote_ip: str, headers: Dict[str, str]) -> Tuple[bool, str]:
    # Con tunnel local, el origen al dashboard puede ser localhost (cloudflared local).
    if remote_ip in {"127.0.0.1", "::1"}:
        return True, "localhost_tunnel"

    if is_cloudflare_edge_ip(remote_ip):
        if headers.get("cf-ray") or headers.get("CF-Ray"):
            return True, "cloudflare_edge"
        return False, "missing_cf_ray_header"
    return False, "remote_ip_not_in_cloudflare_ranges"
