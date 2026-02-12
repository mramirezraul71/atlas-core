"""Gateway models: GatewayMode, GatewayStatus, WorkerTarget."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


GATEWAY_MODES = ("auto", "cloudflare", "tailscale", "ssh", "lan")


@dataclass
class WorkerTarget:
    mode: str
    base_url: str
    node_id: Optional[str] = None
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class GatewayStatus:
    enabled: bool
    mode: str
    tools: Dict[str, bool]  # cloudflared, tailscale, ssh
    last_success_ts: Optional[str] = None
    last_success_mode: Optional[str] = None
    last_success_target: Optional[str] = None
    recommendations: List[str] = field(default_factory=list)
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "tools": self.tools,
            "last_success_ts": self.last_success_ts,
            "last_success_mode": self.last_success_mode,
            "last_success_target": self.last_success_target,
            "recommendations": self.recommendations,
            "error": self.error,
        }
