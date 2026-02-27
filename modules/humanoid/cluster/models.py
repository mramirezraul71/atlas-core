"""Cluster models: Node, Heartbeat, RemoteCall, RoutingDecision."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Node:
    node_id: str
    role: str  # hq | worker
    base_url: str
    capabilities: Dict[str, bool]  # hands, vision, web, voice, llm
    last_seen_ts: Optional[str] = None
    health_score: int = 0
    status: str = "offline"  # online | offline | degraded
    tags: Dict[str, str] = field(default_factory=dict)
    created_ts: Optional[str] = None
    updated_ts: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        import json
        return {
            "node_id": self.node_id,
            "role": self.role,
            "base_url": self.base_url,
            "capabilities": self.capabilities,
            "last_seen_ts": self.last_seen_ts,
            "health_score": self.health_score,
            "status": self.status,
            "tags": self.tags,
            "created_ts": self.created_ts,
            "updated_ts": self.updated_ts,
        }


@dataclass
class Heartbeat:
    node_id: str
    capabilities: Dict[str, bool]
    health: Dict[str, Any]  # score, checks
    version: str = ""
    channel: str = "canary"


@dataclass
class RemoteCall:
    target: str  # hands | web | vision | voice
    payload: Dict[str, Any]
    correlation_id: Optional[str] = None
    timeout_sec: int = 30


@dataclass
class RoutingDecision:
    route: str  # local | remote
    node_id: Optional[str] = None
    base_url: Optional[str] = None
    reason: str = ""
