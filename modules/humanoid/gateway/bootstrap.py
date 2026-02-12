"""Bootstrap: probe worker, register to HQ. Audit + fallbacks."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

from . import health
from . import selector
from . import store
from . import audit as gateway_audit
from .models import GatewayStatus

HQ_BASE_URL = os.getenv("ATLAS_HQ_URL", "").strip()  # HQ URL for register call (from worker perspective often same host)


def _register_worker_at_hq(hq_base_url: str, node_id: str, base_url: str, capabilities: Dict[str, bool], role: str = "worker") -> Dict[str, Any]:
    """POST /cluster/node/register at HQ."""
    import json
    import urllib.request
    import urllib.error
    url = f"{hq_base_url.rstrip('/')}/cluster/node/register"
    body = json.dumps({"node_id": node_id, "role": role, "base_url": base_url, "capabilities": capabilities or {}}).encode("utf-8")
    try:
        req = urllib.request.Request(url, data=body, method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=15) as r:
            data = json.loads(r.read().decode())
            return {"ok": True, "data": data}
    except urllib.error.HTTPError as e:
        try:
            data = json.loads(e.read().decode())
        except Exception:
            data = {}
        return {"ok": False, "error": data.get("error", str(e)), "status": e.code}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def bootstrap(hq_base_url: Optional[str] = None, node_id: Optional[str] = None, mode_override: Optional[str] = None) -> Dict[str, Any]:
    """
    Resolve worker URL, probe health with retries, register at HQ.
    If one mode fails, try next (auto only). Audit gateway_start/fail/success.
    """
    t0 = time.perf_counter()
    hq = hq_base_url or HQ_BASE_URL or "http://127.0.0.1:8791"
    nid = node_id or os.getenv("CLUSTER_NODE_ID", "worker-1")
    capabilities = {"hands": True, "web": True, "vision": True, "voice": True, "llm": True}
    recommendations: List[str] = []
    current_mode = store.get_mode()
    mode = mode_override or current_mode
    tried: List[str] = []
    if mode == "auto":
        for name in selector._order():
            target = selector._resolve_one(name)
            if not target:
                continue
            tried.append(name)
            gateway_audit.log_gateway_event("bootstrap_attempt", False, {"mode": name, "target": target.base_url}, None, 0)
            probed = health.probe_worker_with_retries(target.base_url)
            if probed.get("ok"):
                reg = _register_worker_at_hq(hq, nid, target.base_url, capabilities)
                ms = int((time.perf_counter() - t0) * 1000)
                if reg.get("ok"):
                    store.set_last_success(name, target.base_url)
                    gateway_audit.log_gateway_event("gateway_success", True, {"mode": name, "target": target.base_url}, None, ms)
                    return {"ok": True, "mode": name, "target": target.base_url, "score": probed.get("score"), "ms": ms, "registered": True}
                gateway_audit.log_gateway_event("gateway_fail", False, {"mode": name}, reg.get("error"), ms)
                recommendations.append(f"Register failed for {name}: {reg.get('error')}")
            else:
                recommendations.append(f"{name}: {probed.get('error', 'probe failed')}")
        ms = int((time.perf_counter() - t0) * 1000)
        gateway_audit.log_gateway_event("gateway_fail", False, {"tried": tried, "recommendations": recommendations}, "all modes failed", ms)
        return {"ok": False, "error": "all modes failed", "tried": tried, "recommendations": recommendations, "ms": ms}
    target = selector.resolve_worker_url(mode)
    if not target:
        ms = int((time.perf_counter() - t0) * 1000)
        gateway_audit.log_gateway_event("gateway_fail", False, {"mode": mode}, "no_gateway_config", ms)
        return {"ok": False, "error": "no_gateway_config", "mode": mode, "ms": ms}
    gateway_audit.log_gateway_event("bootstrap_attempt", False, {"mode": mode, "target": target.base_url}, None, 0)
    probed = health.probe_worker_with_retries(target.base_url)
    ms = int((time.perf_counter() - t0) * 1000)
    if not probed.get("ok"):
        gateway_audit.log_gateway_event("gateway_fail", False, {"mode": mode}, probed.get("error"), ms)
        return {"ok": False, "error": probed.get("error", "probe failed"), "mode": mode, "ms": ms, "recommendations": [probed.get("error", "")]}
    reg = _register_worker_at_hq(hq, nid, target.base_url, capabilities)
    if reg.get("ok"):
        store.set_last_success(mode, target.base_url)
        gateway_audit.log_gateway_event("gateway_success", True, {"mode": mode, "target": target.base_url}, None, ms)
        return {"ok": True, "mode": mode, "target": target.base_url, "score": probed.get("score"), "ms": ms, "registered": True}
    gateway_audit.log_gateway_event("gateway_fail", False, {"mode": mode}, reg.get("error"), ms)
    return {"ok": False, "error": reg.get("error", "register failed"), "mode": mode, "ms": ms}
