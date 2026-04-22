from __future__ import annotations

from dataclasses import dataclass, field

from .score_breakdown import ScoreBreakdown


@dataclass(frozen=True)
class CandidateOpportunity:
    candidate_id: str
    snapshot_id: str
    symbol: str
    direction: str
    thesis: str
    score: float
    score_breakdown: ScoreBreakdown
    time_horizon_minutes: int
    max_risk_pct: float
    expected_rr: float
    tags: tuple[str, ...] = field(default_factory=tuple)
    meta: dict[str, object] = field(default_factory=dict)

