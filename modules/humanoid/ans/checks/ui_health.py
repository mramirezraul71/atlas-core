"""UI health check."""
from __future__ import annotations

import os
import urllib.request


def run() -> dict:
    try:
        port = int(os.getenv("ACTIVE_PORT", "8791"))
        url = f"http://127.0.0.1:{port}/ui"
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=5) as resp:
            ok = resp.status == 200
        return {"ok": ok, "check_id": "ui_health", "message": "ok" if ok else "fail", "details": {}, "severity": "low"}
    except Exception as e:
        return {"ok": False, "check_id": "ui_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
