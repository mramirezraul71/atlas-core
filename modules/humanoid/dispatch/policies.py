"""Per-node allowlists for remote execution (future: restrict which nodes can run what)."""
from __future__ import annotations

import os
from typing import List, Set


def allowed_remote_actions(node_id: str) -> Set[str]:
    """Actions allowed for this node. Default: hands, web, vision, voice."""
    v = os.getenv("CLUSTER_ALLOWED_ACTIONS", "hands,web,vision,voice")
    return set(x.strip() for x in v.split(",") if x.strip())


def is_action_allowed_for_node(node_id: str, action: str) -> bool:
    return action in allowed_remote_actions(node_id)
