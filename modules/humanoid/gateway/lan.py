"""LAN: simple target resolver from env."""
from __future__ import annotations

import os
from typing import Optional


def get_worker_url() -> Optional[str]:
    url = os.getenv("LAN_WORKER_URL", "").strip()
    return url.rstrip("/") if url else None


def status() -> dict:
    url = get_worker_url()
    enabled = os.getenv("LAN_ENABLED", "").strip().lower() in ("1", "true", "yes")
    return {"ok": bool(url), "available": bool(url), "enabled": enabled, "url": url or ""}
