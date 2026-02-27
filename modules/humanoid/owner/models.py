"""Owner control models: OwnerSession, EmergencyState."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class OwnerSession:
    """Owner session with TTL."""
    token: str
    actor: str
    method: str  # ui | telegram | voice | api
    created_at: float
    ttl_seconds: int
    expires_at: Optional[str] = None


@dataclass
class EmergencyState:
    """Emergency mode state with block flags."""
    enabled: bool
    block_deploy: bool
    block_remote_exec: bool
    block_shell: bool
    allow_status: bool
    reason: Optional[str] = None
