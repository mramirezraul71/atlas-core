from __future__ import annotations

import pytest

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import (
    ScanConfig,
    build_default_scan_config_offline,
    validate_scan_config,
)
from atlas_scanner.exceptions import ScannerInputError


def test_build_default_scan_config_offline_returns_valid_config() -> None:
    scan_config = build_default_scan_config_offline()

    assert isinstance(scan_config, ScanConfig)
    assert isinstance(scan_config.universe_name, str)
    assert scan_config.universe_name != ""

    assert isinstance(scan_config.max_final_candidates, int)
    assert scan_config.max_final_candidates >= 0
    assert isinstance(scan_config.default_max_per_family, int)
    assert scan_config.default_max_per_family >= 0

    assert isinstance(scan_config.max_per_family, dict)
    assert all(limit >= 0 for limit in scan_config.max_per_family.values())

    assert scan_config.weights == SCORING_CONFIG.weights
    assert scan_config.rules == SCORING_CONFIG.rules
    assert (
        scan_config.max_final_candidates == SCORING_CONFIG.rules.max_final_candidates
    )
    assert (
        scan_config.default_max_per_family
        == SCORING_CONFIG.rules.default_max_per_family
    )


def test_build_default_scan_config_offline_is_deterministic() -> None:
    left = build_default_scan_config_offline()
    right = build_default_scan_config_offline()

    assert left == right


def test_validate_scan_config_rejects_negative_limits() -> None:
    invalid = ScanConfig(
        universe_name="default",
        weights=SCORING_CONFIG.weights,
        rules=SCORING_CONFIG.rules,
        max_final_candidates=-1,
        default_max_per_family=SCORING_CONFIG.rules.default_max_per_family,
        max_per_family={"IRON_CONDOR": 2},
    )

    with pytest.raises(ScannerInputError):
        validate_scan_config(invalid)

