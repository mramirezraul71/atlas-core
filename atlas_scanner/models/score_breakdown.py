from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ScoreComponent:
    name: str
    contribution: float
    weight: float
    raw: float


@dataclass(frozen=True)
class ScoreBreakdown:
    raw_score: float
    normalized_score: float
    components: tuple[ScoreComponent, ...] = field(default_factory=tuple)

