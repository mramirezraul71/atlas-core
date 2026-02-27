"""Deploy state and response models. Minimal dataclasses/dicts for JSON persistence."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class LastDeploy:
    ts: Optional[str] = None
    ref: Optional[str] = None
    result: Optional[str] = None  # success | rollback
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"ts": self.ts, "ref": self.ref, "result": self.result, "error": self.error}

    @classmethod
    def from_dict(cls, d: Optional[Dict[str, Any]]) -> "LastDeploy":
        if not d:
            return cls()
        return cls(ts=d.get("ts"), ref=d.get("ref"), result=d.get("result"), error=d.get("error"))


@dataclass
class DeployState:
    mode: str = "single"
    active_port: int = 8791
    staging_port: int = 8792
    active_pid: Optional[int] = None
    staging_pid: Optional[int] = None
    last_deploy: Optional[Dict[str, Any]] = None
    last_health: Optional[Dict[str, Any]] = None
    auto_switch_on_health: bool = True
    last_switch_ts: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "active_port": self.active_port,
            "staging_port": self.staging_port,
            "active_pid": self.active_pid,
            "staging_pid": self.staging_pid,
            "last_deploy": self.last_deploy,
            "last_health": self.last_health,
            "auto_switch_on_health": self.auto_switch_on_health,
            "last_switch_ts": self.last_switch_ts,
        }