"""ANS data models."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class CheckResult:
    ok: bool
    check_id: str
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    severity: str = "low"


@dataclass
class Issue:
    check_id: str
    severity: str  # low|med|high|critical
    fingerprint: str
    message: str
    evidence: Dict[str, Any] = field(default_factory=dict)
    suggested_heals: List[str] = field(default_factory=list)


@dataclass
class Action:
    heal_id: str
    params: Dict[str, Any] = field(default_factory=dict)
    safe: bool = True
    requires_approval: bool = False


@dataclass
class HealResult:
    ok: bool
    heal_id: str
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None


@dataclass
class Incident:
    id: str
    fingerprint: str
    check_id: str
    severity: str
    message: str
    status: str  # open|resolved|acknowledged
    evidence: Dict[str, Any] = field(default_factory=dict)
    suggested_heals: List[str] = field(default_factory=list)
    actions_taken: List[Dict[str, Any]] = field(default_factory=list)
    created_at: str = ""
    resolved_at: Optional[str] = None


@dataclass
class PolicyDecision:
    allow: bool
    reason: str
    requires_approval: bool = False
