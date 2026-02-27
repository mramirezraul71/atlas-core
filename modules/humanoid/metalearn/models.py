"""Meta-Learning data models."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class FeedbackEvent:
    """Single feedback event for learning."""
    ts: str
    action_type: str
    risk_level: str
    decision: str  # approve | reject | auto | failed | expired
    outcome: str   # ok | fail
    latency_ms: Optional[int] = None
    node_id: Optional[str] = None
    model_used: Optional[str] = None
    features_json: Optional[Dict[str, Any]] = None
    correlation_id: Optional[str] = None
    thread_id: Optional[str] = None
    task_id: Optional[str] = None
    source: str = "unknown"  # approval | ga | deploy | scheduler | router


@dataclass
class LearnedRule:
    """Inferred rule: conditions -> bounded adjustments."""
    rule_id: str
    conditions: Dict[str, Any]
    risk_adjust: float
    router_hint: Optional[str] = None
    canary_hint: Optional[float] = None
    approve_rate: float = 0.0
    success_rate: float = 0.0
    sample_count: int = 0
    created_ts: str = ""


@dataclass
class ScoreAdjust:
    """Bounded score adjustment for an action type."""
    action_type: str
    risk_shift: float
    evidence_count: int


@dataclass
class RouterStats:
    """Per-route/model stats for routing hints."""
    route: str
    model_family: str
    success_count: int
    fail_count: int
    avg_latency_ms: float
    sample_count: int
