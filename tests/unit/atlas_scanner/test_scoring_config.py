from __future__ import annotations

from atlas_scanner.config import SCORING_CONFIG


def test_scoring_config_defaults() -> None:
    assert SCORING_CONFIG.config_version == "scanner-s0-v1"
    assert SCORING_CONFIG.score_scale.minimum == 0.0
    assert SCORING_CONFIG.score_scale.maximum == 1.0
    assert SCORING_CONFIG.thresholds.promotion_threshold == 0.60

