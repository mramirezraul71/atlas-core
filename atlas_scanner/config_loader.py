from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
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
class OfflineScoringWeights:
    liquidity: float = 0.40
    price: float = 0.20
    event_risk: float = 0.25
    spread: float = 0.15


@dataclass(frozen=True)
class OfflineScoringThresholds:
    liquidity_lower: float = 0.0
    liquidity_upper: float = 1.0
    price_lower: float = 5.0
    price_upper: float = 500.0
    event_risk_lower: float = 0.0
    event_risk_upper: float = 1.0
    spread_lower: float = 0.0
    spread_upper: float = 0.10


@dataclass(frozen=True)
class ComponentWeights:
    vol: float = 0.40
    gamma: float = 0.20
    oi_flow: float = 0.15
    price: float = 0.20
    macro: float = 0.05

    def normalized(self) -> ComponentWeights:
        total = self.vol + self.gamma + self.oi_flow + self.price + self.macro
        if total <= 0:
            return ComponentWeights(0.0, 0.0, 0.0, 0.0, 0.0)
        return ComponentWeights(
            vol=self.vol / total,
            gamma=self.gamma / total,
            oi_flow=self.oi_flow / total,
            price=self.price / total,
            macro=self.macro / total,
        )


@dataclass(frozen=True)
class VolScoringConfig:
    iv_rank_min: float = 40.0
    iv_rank_max: float = 85.0
    vrp_min: float = 0.0


@dataclass(frozen=True)
class GammaScoringConfig:
    max_pain_distance_pct_max: float = 1.5
    net_gex_negative_only: bool = True


@dataclass(frozen=True)
class OIFlowScoringConfig:
    oi_change_min_pct: float = 0.0
    call_put_volume_ratio_min: float = 0.0
    call_put_volume_ratio_max: float = 999.0


@dataclass(frozen=True)
class PriceScoringConfig:
    adx_ranging_max: float = 25.0
    adx_trending_min: float = 30.0


@dataclass(frozen=True)
class MacroScoringConfig:
    seasonal_factor_min: float = 0.0
    seasonal_factor_max: float = 2.0
    block_on_event_risk_high: bool = True


@dataclass(frozen=True)
class OfflineScoringConfig:
    component_weights: ComponentWeights = field(default_factory=ComponentWeights)
    vol: VolScoringConfig = field(default_factory=VolScoringConfig)
    gamma: GammaScoringConfig = field(default_factory=GammaScoringConfig)
    oi_flow: OIFlowScoringConfig = field(default_factory=OIFlowScoringConfig)
    price: PriceScoringConfig = field(default_factory=PriceScoringConfig)
    macro: MacroScoringConfig = field(default_factory=MacroScoringConfig)
    weights: OfflineScoringWeights = field(default_factory=OfflineScoringWeights)
    thresholds: OfflineScoringThresholds = field(default_factory=OfflineScoringThresholds)
    meta: dict[str, Any] | None = None


DEFAULT_OFFLINE_SCORING_CONFIG = OfflineScoringConfig()


@dataclass(frozen=True)
class ScanConfig:
    universe_name: str
    weights: ScoringWeights
    rules: ScoringRules
    max_final_candidates: int
    default_max_per_family: int
    max_per_family: Mapping[str, int]
    scoring: OfflineScoringConfig = DEFAULT_OFFLINE_SCORING_CONFIG
    active_filters: tuple[str, ...] = DEFAULT_ACTIVE_FILTERS
    active_features: tuple[str, ...] = DEFAULT_ACTIVE_FEATURES


def validate_offline_scoring_config(scoring: OfflineScoringConfig) -> None:
    runtime_weight_sum = (
        scoring.weights.liquidity
        + scoring.weights.price
        + scoring.weights.event_risk
        + scoring.weights.spread
    )
    if runtime_weight_sum <= 0:
        raise ScannerInputError("offline scoring runtime weights sum must be > 0")

    component_weight_sum = (
        scoring.component_weights.vol
        + scoring.component_weights.gamma
        + scoring.component_weights.oi_flow
        + scoring.component_weights.price
        + scoring.component_weights.macro
    )
    if component_weight_sum <= 0:
        raise ScannerInputError("offline scoring component weights sum must be > 0")


def validate_scan_config(scan_config: ScanConfig) -> None:
    if not scan_config.universe_name.strip():
        raise ScannerInputError("universe_name must be a non-empty string")
    if scan_config.max_final_candidates < 0:
        raise ScannerInputError("max_final_candidates must be >= 0")
    if scan_config.default_max_per_family < 0:
        raise ScannerInputError("default_max_per_family must be >= 0")
    if any(limit < 0 for limit in scan_config.max_per_family.values()):
        raise ScannerInputError("max_per_family values must be >= 0")
    validate_offline_scoring_config(scan_config.scoring)


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

