from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .score_breakdown import ScoreBreakdown


@dataclass(frozen=True)
class CandidateOpportunity:
    symbol: str
    underlying_type: str
    strategy_family: str
    direction: str
    thesis: str
    expiry: str
    strike_range: Tuple[float, float]
    normalized_score: float
    score_breakdown: ScoreBreakdown
    regime_id: str
    entry_reason: str
    feature_snapshot: Tuple[Tuple[str, str], ...]
    event_risk: bool = False

