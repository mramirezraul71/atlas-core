"""HTTP client for cluster: signed requests, timeouts, retries."""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Optional

from . import auth as cluster_auth
from . import registry

TIMEOUT_DEFAULT = 30
RETRIES = 2


def _httpx_post(base_url: str, path: str, body: Dict[str, Any], headers: Dict[str, str], timeout_sec: int) -> Dict[str, Any]:
    try:
        import urllib.request
        import urllib.error
        url = f"{base_url.rstrip('/')}{path}"
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="POST", headers={**headers, "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout_sec) as r:
            raw = r.read().decode()
            return {"ok": True, "status": r.status, "data": json.loads(raw) if raw else {}}
    except urllib.error.HTTPError as e:
        try:
            body = e.read().decode()
            data = json.loads(body) if body else {}
        except Exception:
            data = {}
        return {"ok": False, "status": e.code, "error": getattr(e, "reason", str(e)), "data": data}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _httpx_get(base_url: str, path: str, headers: Dict[str, str], timeout_sec: int) -> Dict[str, Any]:
    try:
        import urllib.request
        import urllib.error
        url = f"{base_url.rstrip('/')}{path}"
        req = urllib.request.Request(url, method="GET", headers=headers)
        with urllib.request.urlopen(req, timeout=timeout_sec) as r:
            raw = r.read().decode()
            return {"ok": True, "status": r.status, "data": json.loads(raw) if raw else {}}
    except urllib.error.HTTPError as e:
        try:
            body = e.read().decode()
            data = json.loads(body) if body else {}
        except Exception:
            data = {}
        return {"ok": False, "status": e.code, "error": getattr(e, "reason", str(e)), "data": data}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _signed_headers(base_url: str, method: str, path: str, body: Optional[bytes] = None) -> Dict[str, str]:
    nid = registry.node_id()
    ts, nonce, sig = cluster_auth.sign_request(nid, method, path, body)
    return {
        "X-Atlas-Node": nid,
        "X-Atlas-Ts": ts,
        "X-Atlas-Nonce": nonce,
        "X-Atlas-Sign": sig,
    }


def post_json(base_url: str, path: str, body: Dict[str, Any], timeout_sec: int = TIMEOUT_DEFAULT, signed: bool = True) -> Dict[str, Any]:
    """POST JSON to cluster node. If signed=True, add HMAC headers."""
    data = json.dumps(body).encode("utf-8")
    headers = _signed_headers(base_url, "POST", path, data) if signed else {}
    for attempt in range(RETRIES + 1):
        out = _httpx_post(base_url, path, body, headers, timeout_sec)
        if out.get("ok") or attempt == RETRIES:
            return out
        time.sleep(0.5)
    return out


def get_json(base_url: str, path: str, timeout_sec: int = TIMEOUT_DEFAULT, signed: bool = True) -> Dict[str, Any]:
    """GET from cluster node. If signed=True, add HMAC headers."""
    headers = _signed_headers(base_url, "GET", path, None) if signed else {}
    return _httpx_get(base_url, path, headers, timeout_sec)
