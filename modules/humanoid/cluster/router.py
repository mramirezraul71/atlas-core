"""Route task to local or best remote node by capabilities and health."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

from . import registry
from . import db as cluster_db


def default_route() -> str:
    return (os.getenv("CLUSTER_DEFAULT_ROUTE") or "local").strip().lower()


def route_decision(
    task: str,
    prefer_remote: bool = False,
    require_capability: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Decide where to run: local | remote.
    Returns {route, node_id?, base_url?, reason}.
    """
    if not registry.cluster_enabled():
        return {"route": "local", "reason": "cluster_disabled"}
    nodes = [n for n in cluster_db.list_nodes() if n.get("status") == "online"]
    if not nodes:
        return {"route": "local", "reason": "no_online_nodes"}
    if require_capability:
        nodes = [n for n in nodes if (n.get("capabilities") or {}).get(require_capability)]
        if not nodes:
            return {"route": "local", "reason": f"no_node_with_{require_capability}"}
    if not prefer_remote and default_route() != "remote_preferred":
        return {"route": "local", "reason": "default_local"}
    best = max(nodes, key=lambda n: (n.get("health_score") or 0, -len(n.get("tags") or {})))
    return {
        "route": "remote",
        "node_id": best["node_id"],
        "base_url": best["base_url"],
        "reason": "best_health",
    }
