from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Literal


@dataclass(frozen=True)
class StrikeLevel:
    strike: float
    kind: Literal["CALL_WALL", "PUT_WALL", "STRUCTURAL", "OTHER"]
    size: float | None = None
    note: str | None = None


@dataclass(frozen=True)
class PriceLevel:
    level: float
    role: Literal["RESISTANCE", "SUPPORT", "WALL", "FLIP", "OTHER"]
    label: str | None = None
    note: str | None = None


@dataclass(frozen=True)
class MacroEvent:
    name: str
    time: datetime | None
    impact: Literal["LOW", "MEDIUM", "HIGH", "CRITICAL"]
    category: Literal["MACRO", "EARNINGS", "GEOPOLITICAL", "OTHER"]
    symbol_scope: str | None = None


@dataclass(frozen=True)
class VolFeatures:
    iv_rank_20d: float | None = None
    iv_rank_50d: float | None = None
    iv_rank_100d: float | None = None
    iv_percentile: float | None = None
    vrp_5d: float | None = None
    vrp_10d: float | None = None
    vrp_20d: float | None = None
    vrp_60d: float | None = None
    term_structure_slope: float | None = None
    term_structure_curvature: float | None = None
    skew_25d: float | None = None


@dataclass(frozen=True)
class GammaFeatures:
    net_gex: float | None = None
    gamma_flip_agg: float | None = None
    gamma_flip_weekly: float | None = None
    distance_to_flip_agg_pct: float | None = None
    distance_to_flip_weekly_pct: float | None = None
    call_wall_nearest: StrikeLevel | None = None
    put_wall_nearest: StrikeLevel | None = None
    max_pain_level: float | None = None


@dataclass(frozen=True)
class OIFlowFeatures:
    oi_change_1d_pct: float | None = None
    call_put_volume_ratio: float | None = None
    volume_imbalance: float | None = None
    highest_oi_strikes: tuple[StrikeLevel, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class PriceFeatures:
    adx_14: float | None = None
    ema_short: float | None = None
    ema_long: float | None = None
    trend_state: Literal["RANGING", "TREND_UP", "TREND_DOWN", "UNKNOWN"] = "UNKNOWN"
    distance_to_vwap: float | None = None
    distance_to_poc: float | None = None
    recent_candle_pattern: str | None = None


@dataclass(frozen=True)
class MacroFeatures:
    regime_id: str | None = None
    macro_regime: str | None = None
    seasonal_score: float | None = None
    macro_blocker: bool | None = None


@dataclass(frozen=True)
class StrategyCandidate:
    strategy_type: Literal[
        "IRON_CONDOR",
        "DEBIT_SPREAD",
        "CREDIT_SPREAD",
        "CALENDAR",
        "OTHER",
    ]
    score: float
    context_match: bool
    notes: str | None = None


@dataclass(frozen=True)
class MarketContextSnapshot:
    """Structured market context emitted by Scanner Pro."""

    symbol: str
    as_of: datetime
    session_tags: tuple[str, ...] = field(default_factory=tuple)
    gamma_flip_agg: float | None = None
    gamma_flip_weekly: float | None = None
    distance_to_gamma_flip_agg: float | None = None
    distance_to_gamma_flip_weekly: float | None = None
    call_walls: tuple[StrikeLevel, ...] = field(default_factory=tuple)
    put_walls: tuple[StrikeLevel, ...] = field(default_factory=tuple)
    max_pain_level: float | None = None
    spot: float = 0.0
    prev_close: float | None = None
    expected_move_lower: float | None = None
    expected_move_upper: float | None = None
    key_levels: tuple[PriceLevel, ...] = field(default_factory=tuple)
    vix: float | None = None
    iv: float | None = None
    iv_rank_20d: float | None = None
    iv_rank_50d: float | None = None
    iv_rank_100d: float | None = None
    iv_percentile: float | None = None
    macro_events: tuple[MacroEvent, ...] = field(default_factory=tuple)
    regime_id: str | None = None
    macro_regime: str | None = None
    session_params: SessionParams | None = None
    meta: dict[str, Any] | None = None


@dataclass(frozen=True)
class CandidateOpportunity:
    """Scanner Pro candidate ready for strategy handoff."""

    symbol: str
    as_of: datetime
    asset_type: Literal["INDEX", "ETF", "STOCK", "FUTURE", "OTHER"]
    dte_candidates: tuple[int, ...] = field(default_factory=tuple)
    dte_preferred: int | None = None
    expiries: tuple[str, ...] = field(default_factory=tuple)
    strikes_relevant: tuple[StrikeLevel, ...] = field(default_factory=tuple)
    vol_features: VolFeatures = field(default_factory=VolFeatures)
    gamma_features: GammaFeatures = field(default_factory=GammaFeatures)
    oi_flow_features: OIFlowFeatures = field(default_factory=OIFlowFeatures)
    price_features: PriceFeatures = field(default_factory=PriceFeatures)
    macro_features: MacroFeatures = field(default_factory=MacroFeatures)
    total_score: float = 0.0
    component_scores: dict[str, float] = field(default_factory=dict)
    weights_effective: dict[str, float] = field(default_factory=dict)
    strategy_candidates: tuple[StrategyCandidate, ...] = field(default_factory=tuple)
    strengths: tuple[str, ...] = field(default_factory=tuple)
    penalties: tuple[str, ...] = field(default_factory=tuple)
    explanation: str = ""
    entry_reason: str | None = None
    market_context: MarketContextSnapshot | None = None
    meta: dict[str, Any] | None = None


@dataclass(frozen=True)
class SessionParams:
    """Operational session parameters used by Scanner Pro."""

    market_hours_gate: datetime | None = None
    max_open_positions_recommended: int | None = None
    score_min: float | None = None
    bias: Literal["LONG", "SHORT", "NEUTRAL", "DEFENSIVE", "UNKNOWN"] = "UNKNOWN"
    stop_structural_intraday: float | None = None
    auton_mode: str | None = None
    executor_mode: str | None = None
    fail_safe_state: str | None = None
    notes: tuple[str, ...] = field(default_factory=tuple)
    meta: dict[str, Any] | None = None

