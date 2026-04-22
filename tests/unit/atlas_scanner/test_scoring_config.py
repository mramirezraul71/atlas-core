from __future__ import annotations

from atlas_scanner.config import SCORING_CONFIG


def test_scoring_config_defaults() -> None:
    assert SCORING_CONFIG.config_version == "scanner-s0-v1"
    assert SCORING_CONFIG.rules.promotion_threshold == 0.60
    assert SCORING_CONFIG.rules.max_final_candidates == 8
    assert SCORING_CONFIG.rules.default_max_per_family == 1

