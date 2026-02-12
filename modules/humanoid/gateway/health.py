"""Health probe + scoring + retries for worker."""
from __future__ import annotations

import json
import os
import time
import urllib.request
import urllib.error
from typing import Any, Dict, Optional

PROBE_TIMEOUT_SEC = 10


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def probe_ping(base_url: str) -> Dict[str, Any]:
    """GET /cluster/ping. Returns {ok, error}."""
    url = f"{base_url.rstrip('/')}/cluster/ping"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=PROBE_TIMEOUT_SEC) as r:
            raw = r.read().decode()
            if r.status == 200:
                data = json.loads(raw) if raw else {}
                return {"ok": data.get("data", {}).get("pong", False), "error": None}
            return {"ok": False, "error": f"status {r.status}"}
    except urllib.error.URLError as e:
        return {"ok": False, "error": str(e.reason) if getattr(e, "reason", None) else str(e)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def probe_health(base_url: str) -> Dict[str, Any]:
    """GET /health. Returns {ok, score, error}."""
    url = f"{base_url.rstrip('/')}/health"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=PROBE_TIMEOUT_SEC) as r:
            raw = r.read().decode()
            data = json.loads(raw) if raw else {}
            score = data.get("score", 0)
            ok = data.get("ok", False) and score >= _env_int("GATEWAY_HEALTH_MIN_SCORE", 75)
            return {"ok": ok, "score": score, "error": None if ok else f"score {score} < min"}
    except Exception as e:
        return {"ok": False, "score": 0, "error": str(e)}


def probe_worker_with_retries(base_url: str) -> Dict[str, Any]:
    """Ping + health with retries. Returns {ok, score, error, attempts}."""
    retry_sec = _env_int("GATEWAY_BOOTSTRAP_RETRY_SECONDS", 5)
    max_retries = _env_int("GATEWAY_BOOTSTRAP_MAX_RETRIES", 10)
    min_score = _env_int("GATEWAY_HEALTH_MIN_SCORE", 75)
    last_error = None
    for attempt in range(max_retries):
        ping = probe_ping(base_url)
        if not ping.get("ok"):
            last_error = ping.get("error", "ping failed")
            time.sleep(retry_sec)
            continue
        health = probe_health(base_url)
        if health.get("ok") and (health.get("score") or 0) >= min_score:
            return {"ok": True, "score": health.get("score"), "error": None, "attempts": attempt + 1}
        last_error = health.get("error") or f"score {health.get('score', 0)} < {min_score}"
        time.sleep(retry_sec)
    return {"ok": False, "score": 0, "error": last_error, "attempts": max_retries}
