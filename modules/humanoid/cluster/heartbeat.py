"""Heartbeat: worker sends to HQ; HQ marks online/offline."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

from . import db as cluster_db
from . import registry
from .remote_client import post_json


def heartbeat_interval_sec() -> int:
    try:
        return int(os.getenv("CLUSTER_HEARTBEAT_SECONDS", "5") or 5)
    except (TypeError, ValueError):
        return 5


def receive_heartbeat(
    node_id_arg: str,
    capabilities: Dict[str, bool],
    health: Dict[str, Any],
    version: str = "",
    channel: str = "canary",
    base_url: Optional[str] = None,
) -> Dict[str, Any]:
    """HQ: process incoming heartbeat from worker. If node unknown, register with base_url from request."""
    if not registry.cluster_enabled():
        return {"ok": False, "error": "CLUSTER_ENABLED=false"}
    score = int(health.get("score", 0)) if isinstance(health.get("score"), (int, float)) else 0
    status = "online" if score >= 50 else "degraded"
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    node = cluster_db.get_node(node_id_arg)
    if not node and base_url:
        cluster_db.upsert_node(node_id_arg, "worker", base_url, capabilities or {}, now, score, status, {})
        node = cluster_db.get_node(node_id_arg)
    if node:
        cluster_db.upsert_node(
            node_id=node_id_arg,
            role=node["role"],
            base_url=node["base_url"],
            capabilities=capabilities,
            last_seen_ts=now,
            health_score=score,
            status=status,
            tags=node.get("tags") or {},
        )
    cluster_db.log_event(node_id_arg, "heartbeat", {"score": score, "status": status})
    return {"ok": True, "received": True}


def send_heartbeat_to_hq(hq_base_url: str) -> Dict[str, Any]:
    """Worker: send own heartbeat to HQ."""
    if not registry.cluster_enabled():
        return {"ok": False, "error": "CLUSTER_ENABLED=false"}
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        from modules.humanoid.release import get_version_info
        health = run_health_verbose(base_url=None)
        version_info = get_version_info()
        caps = _local_capabilities()
        body = {
            "node_id": registry.node_id(),
            "capabilities": caps,
            "health": {"score": health.get("score", 0), "checks": health.get("checks", {})},
            "version": version_info.get("version", "0.0.0"),
            "channel": version_info.get("channel", "canary"),
        }
        out = post_json(hq_base_url, "/cluster/heartbeat", body, timeout_sec=10)
        return out
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _local_capabilities() -> Dict[str, bool]:
    v = os.getenv("VISION_ENABLED", "true").strip().lower() in ("1", "true", "yes")
    w = os.getenv("WEB_ENABLED", "true").strip().lower() in ("1", "true", "yes")
    vo = os.getenv("VOICE_ENABLED", "true").strip().lower() in ("1", "true", "yes")
    return {"hands": True, "vision": v, "web": w, "voice": vo, "llm": True}
