"""Mark node offline if heartbeat lost."""
from __future__ import annotations

from .base import heal_result


def run(node_id: str = "", **kwargs) -> dict:
    try:
        from modules.humanoid.cluster import db as cluster_db
        if node_id:
            cluster_db.upsert_node(node_id=node_id, role="worker", base_url="", capabilities={}, status="offline")
        return heal_result(True, "mark_node_offline", f"marked {node_id or 'stale'} offline", {"node_id": node_id})
    except Exception as e:
        return heal_result(False, "mark_node_offline", str(e), {}, str(e))
