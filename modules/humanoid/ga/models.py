"""Governed Autonomy data models."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Finding:
    """Detected opportunity or issue from a signal source."""
    source: str
    kind: str
    path: str
    detail: str
    score: float
    meta: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ActionCandidate:
    """Proposed action derived from a Finding."""
    finding: Finding
    action_type: str
    risk_level: str
    roi_score: int
    evidence_required: List[str]
    payload: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ActionPlan:
    """Plan split: safe vs approval vs deferred."""
    safe: List[ActionCandidate] = field(default_factory=list)
    approvals: List[ActionCandidate] = field(default_factory=list)
    deferred: List[ActionCandidate] = field(default_factory=list)


@dataclass
class ExecutionResult:
    """Result of executing a safe action."""
    action_type: str
    ok: bool
    exit_code: Optional[int] = None
    stdout_snip: Optional[str] = None
    changed_files: List[str] = field(default_factory=list)
    paths: List[str] = field(default_factory=list)
    evidence: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None
    ms: int = 0
