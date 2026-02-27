"""Cluster health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.cluster import registry
        if not registry.cluster_enabled():
            return {"ok": True, "check_id": "cluster_health", "message": "cluster disabled", "details": {}, "severity": "low"}
        from modules.humanoid.cluster import db as cluster_db
        nodes = cluster_db.list_nodes()
        online = sum(1 for n in nodes if n.get("status") == "online")
        return {"ok": True, "check_id": "cluster_health", "message": f"nodes={len(nodes)} online={online}", "details": {"nodes": len(nodes), "online": online}, "severity": "low", "suggested_heals": ["mark_node_offline"] if online < len(nodes) else []}
    except Exception as e:
        return {"ok": False, "check_id": "cluster_health", "message": str(e), "details": {"error": str(e)}, "severity": "low"}
