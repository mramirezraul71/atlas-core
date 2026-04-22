from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple


@dataclass(frozen=True)
class LiquidityThresholds:
    min_liquidity_score: float = 0.40
    min_volume_contracts: int = 100


@dataclass(frozen=True)
class VolatilityThresholds:
    min_iv_rank: float = 0.20
    max_iv_rank: float = 0.85


@dataclass(frozen=True)
class GammaThresholds:
    min_abs_gamma_exposure: float = 0.10


@dataclass(frozen=True)
class PriceThresholds:
    min_price: float = 5.0
    max_price: float = 1000.0


@dataclass(frozen=True)
class RiskThresholds:
    max_risk_pct: float = 0.02
    min_expected_rr: float = 1.5


@dataclass(frozen=True)
class UniverseConfig:
    max_symbols: int = 200
    allow_etf: bool = True
    allow_equity: bool = True


@dataclass(frozen=True)
class SchedulingHints:
    scan_window_minutes: int = 15
    cooldown_minutes: int = 5


@dataclass(frozen=True)
class ScoringWeights:
    liquidity: float = 0.25
    volatility: float = 0.25
    gamma: float = 0.20
    price: float = 0.10
    risk: float = 0.20


@dataclass(frozen=True)
class ScoringRules:
    promotion_threshold: float = 0.60
    max_final_candidates: int = 8
    default_max_per_family: int = 1
    max_per_family: Tuple[Tuple[str, int], ...] = (
        ("IRON_CONDOR", 2),
        ("DEBIT_SPREAD", 1),
    )


@dataclass(frozen=True)
class ScoringConfig:
    config_version: str = "scanner-s0-v1"
    liquidity: LiquidityThresholds = field(default_factory=LiquidityThresholds)
    volatility: VolatilityThresholds = field(default_factory=VolatilityThresholds)
    gamma: GammaThresholds = field(default_factory=GammaThresholds)
    price: PriceThresholds = field(default_factory=PriceThresholds)
    risk: RiskThresholds = field(default_factory=RiskThresholds)
    universe: UniverseConfig = field(default_factory=UniverseConfig)
    scheduling: SchedulingHints = field(default_factory=SchedulingHints)
    weights: ScoringWeights = field(default_factory=ScoringWeights)
    rules: ScoringRules = field(default_factory=ScoringRules)


SCORING_CONFIG: ScoringConfig = ScoringConfig()

