from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .config import SCORING_CONFIG, ScoringConfig, ScoringRules, ScoringWeights
from .exceptions import ScannerInputError

DEFAULT_ACTIVE_FILTERS: tuple[str, ...] = (
    "liquidity",
    "tradability",
    "event_risk",
)

DEFAULT_ACTIVE_FEATURES: tuple[str, ...] = (
    "volatility",
    "gamma",
    "flow",
    "price",
    "macro",
    "visual",
)


@dataclass(frozen=True)
class ScanConfig:
    universe_name: str
    weights: ScoringWeights
    rules: ScoringRules
    max_final_candidates: int
    default_max_per_family: int
    max_per_family: Mapping[str, int]
    active_filters: tuple[str, ...] = DEFAULT_ACTIVE_FILTERS
    active_features: tuple[str, ...] = DEFAULT_ACTIVE_FEATURES


def validate_scan_config(scan_config: ScanConfig) -> None:
    if not scan_config.universe_name.strip():
        raise ScannerInputError("universe_name must be a non-empty string")
    if scan_config.max_final_candidates < 0:
        raise ScannerInputError("max_final_candidates must be >= 0")
    if scan_config.default_max_per_family < 0:
        raise ScannerInputError("default_max_per_family must be >= 0")
    if any(limit < 0 for limit in scan_config.max_per_family.values()):
        raise ScannerInputError("max_per_family values must be >= 0")


def build_scan_config_offline(
    scoring_config: ScoringConfig,
    universe_name: str = "default",
) -> ScanConfig:
    max_per_family = {
        family: limit for family, limit in scoring_config.rules.max_per_family
    }
    scan_config = ScanConfig(
        universe_name=universe_name,
        weights=scoring_config.weights,
        rules=scoring_config.rules,
        max_final_candidates=scoring_config.rules.max_final_candidates,
        default_max_per_family=scoring_config.rules.default_max_per_family,
        max_per_family=max_per_family,
    )
    validate_scan_config(scan_config)
    return scan_config


def build_default_scan_config_offline() -> ScanConfig:
    return build_scan_config_offline(scoring_config=SCORING_CONFIG)

