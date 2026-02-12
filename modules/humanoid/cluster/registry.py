"""Node registry: register, list, update nodes. HQ holds registry."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

from . import db as cluster_db


def cluster_enabled() -> bool:
    return os.getenv("CLUSTER_ENABLED", "").strip().lower() in ("1", "true", "yes")


def node_id() -> str:
    return (os.getenv("CLUSTER_NODE_ID") or "HQ-1").strip()


def node_role() -> str:
    r = (os.getenv("CLUSTER_ROLE") or "hq").strip().lower()
    return r if r in ("hq", "worker") else "hq"


def offline_seconds() -> int:
    try:
        return int(os.getenv("CLUSTER_NODE_OFFLINE_SECONDS", "20") or 20)
    except (TypeError, ValueError):
        return 20


def register_node(
    node_id_arg: str,
    role: str,
    base_url: str,
    capabilities: Optional[Dict[str, bool]] = None,
    tags: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    """Register or update a node. Called by worker or HQ bootstrap."""
    if not cluster_enabled():
        return {"ok": False, "error": "CLUSTER_ENABLED=false"}
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    caps = capabilities or {}
    for k in ("hands", "web", "vision", "voice", "llm"):
        if k not in caps:
            caps[k] = True
    cluster_db.upsert_node(
        node_id=node_id_arg,
        role=role,
        base_url=base_url.rstrip("/"),
        capabilities=caps,
        last_seen_ts=now,
        health_score=100,
        status="online",
        tags=tags or {},
    )
    cluster_db.log_event(node_id_arg, "register", {"base_url": base_url, "role": role})
    return {"ok": True, "node_id": node_id_arg, "status": "online"}


def list_nodes(status_filter: Optional[str] = None) -> List[Dict[str, Any]]:
    return cluster_db.list_nodes(status_filter=status_filter)


def update_node_seen(node_id_arg: str, health_score: int = 0, status: str = "online") -> None:
    node = cluster_db.get_node(node_id_arg)
    if not node:
        return
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    cluster_db.upsert_node(
        node_id=node_id_arg,
        role=node["role"],
        base_url=node["base_url"],
        capabilities=node.get("capabilities") or {},
        last_seen_ts=now,
        health_score=health_score,
        status=status,
        tags=node.get("tags") or {},
    )


def mark_offline_stale() -> None:
    """Mark nodes that haven't been seen in CLUSTER_NODE_OFFLINE_SECONDS."""
    sec = offline_seconds()
    cutoff = time.time() - sec
    for n in cluster_db.list_nodes():
        last = n.get("last_seen_ts")
        if not last:
            continue
        try:
            from datetime import datetime
            dt = datetime.fromisoformat(last.replace("Z", "+00:00"))
            ts = dt.timestamp()
        except Exception:
            continue
        if ts < cutoff and n.get("status") != "offline":
            cluster_db.upsert_node(
                node_id=n["node_id"],
                role=n["role"],
                base_url=n["base_url"],
                capabilities=n.get("capabilities") or {},
                last_seen_ts=last,
                health_score=n.get("health_score", 0),
                status="offline",
                tags=n.get("tags") or {},
            )
            cluster_db.log_event(n["node_id"], "offline", {"reason": "stale_heartbeat"})
