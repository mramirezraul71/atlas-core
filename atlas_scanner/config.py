from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ScoreScale:
    minimum: float = 0.0
    maximum: float = 1.0


@dataclass(frozen=True)
class ScoreThresholds:
    promotion_threshold: float = 0.60


@dataclass(frozen=True)
class ScannerScoringConfig:
    config_version: str = "scanner-s0-v1"
    score_scale: ScoreScale = field(default_factory=ScoreScale)
    thresholds: ScoreThresholds = field(default_factory=ScoreThresholds)


SCORING_CONFIG = ScannerScoringConfig()

