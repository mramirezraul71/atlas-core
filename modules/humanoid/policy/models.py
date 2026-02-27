"""Policy data models: ActorContext, PolicyDecision, PolicyRule."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ActorContext:
    """Who is performing the action: actor id and role."""
    actor: str = "api"
    role: str = "owner"


@dataclass
class PolicyDecision:
    """Result of policy check: allow + reason (deny-by-default in strict)."""
    allow: bool
    reason: str
    details: Optional[Dict[str, Any]] = None


@dataclass
class PolicyRule:
    """Single policy rule: id, scope, condition, action."""
    id: str
    scope: str
    condition: Dict[str, Any]
    action: str  # allow | deny
    meta: Optional[Dict[str, Any]] = None


@dataclass
class PolicyResult:
    """Legacy: alias for PolicyDecision-style result."""
    allowed: bool
    reason: str
    matched_rules: List[str] = field(default_factory=list)
    details: Optional[Dict[str, Any]] = None
