from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Literal, Mapping

RadarTimeframe = Literal["1m", "5m", "15m", "1h", "1d"]


@dataclass(frozen=True)
class MarketPerceptionSnapshot:
    symbol: str
    timeframe: RadarTimeframe
    as_of: datetime
    spot_price: float | None = None
    close_price: float | None = None
    return_pct: float | None = None
    momentum_score: float | None = None
    relative_volume: float | None = None
    volume_acceleration: float | None = None
    diagnostics: Mapping[str, str] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class FlowPerceptionSnapshot:
    symbol: str
    timeframe: RadarTimeframe
    as_of: datetime
    equity_flow_bias: float | None = None
    call_volume: float | None = None
    put_volume: float | None = None
    call_premium: float | None = None
    put_premium: float | None = None
    net_bias: float | None = None
    aggression_score: float | None = None
    dte_distribution: Mapping[str, float] = field(default_factory=dict)
    dte_call_premium: Mapping[str, float] = field(default_factory=dict)
    dte_put_premium: Mapping[str, float] = field(default_factory=dict)
    call_moneyness_distance_pct: float | None = None
    put_moneyness_distance_pct: float | None = None
    post_flow_price_reaction_pct: float | None = None
    net_gamma: float | None = None
    oi_concentration: float | None = None
    iv_context: float | None = None
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    diagnostics: Mapping[str, str] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class OperationalContextSnapshot:
    symbol: str
    as_of: datetime
    market_session: str
    runtime_mode: str = "paper"
    vision_available: bool = False
    operator_present: bool = True
    provider_latency_ms: Mapping[str, int] = field(default_factory=dict)
    provider_status: Mapping[str, str] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class DealerPositioningSnapshot:
    symbol: str
    as_of: datetime
    source: str
    gamma_flip_level: float | None = None
    call_wall: float | None = None
    put_wall: float | None = None
    pinning_zone: tuple[float, float] | None = None
    acceleration_zone: tuple[float, float] | None = None
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class EconomicUpcomingEvent:
    event_name: str
    release_datetime: datetime
    impact_level: Literal["low", "medium", "high"] = "medium"
    affected_sectors: tuple[str, ...] = ()
    consensus: float | None = None
    previous_value: float | None = None


@dataclass(frozen=True)
class EconomicRecentEvent:
    event_name: str
    release_datetime: datetime
    actual_value: float | None = None
    consensus: float | None = None
    surprise: float | None = None
    impact_level: Literal["low", "medium", "high"] = "medium"
    affected_sectors: tuple[str, ...] = ()


@dataclass(frozen=True)
class MacroContextSnapshot:
    scope: str
    as_of: datetime
    source: str
    upcoming_event: str | None = None
    recent_event: str | None = None
    upcoming_events: tuple[EconomicUpcomingEvent, ...] = ()
    recent_events: tuple[EconomicRecentEvent, ...] = ()
    high_impact_flag: bool = False
    calendar_risk_score: float = 0.0
    calendar_volatility_window: bool = False
    sector_sensitivity: float | None = None
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class InstitutionalOwnershipSnapshot:
    symbol: str
    as_of: datetime
    source: str
    ownership_pct: float | None = None
    ownership_delta_pct: float | None = None
    concentration_score: float | None = None
    horizon_bias: Literal["intraday", "swing", "positional"] = "positional"
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class InsiderTradingSnapshot:
    symbol: str
    as_of: datetime
    source: str
    net_insider_value: float | None = None
    buy_sell_ratio: float | None = None
    notable_buyers: tuple[str, ...] = ()
    notable_sellers: tuple[str, ...] = ()
    horizon_bias: Literal["intraday", "swing", "positional"] = "swing"
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PoliticalTradingSnapshot:
    scope: str
    as_of: datetime
    source: str
    net_political_flow: float | None = None
    related_tickers: tuple[str, ...] = ()
    disclosure_lag_days: int | None = None
    horizon_bias: Literal["intraday", "swing", "positional"] = "positional"
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RegulatoryEventSnapshot:
    symbol_or_scope: str
    as_of: datetime
    source: str
    event_type: str | None = None
    severity: Literal["low", "medium", "high", "critical"] = "low"
    overhang_score: float | None = None
    horizon_bias: Literal["intraday", "swing", "positional"] = "swing"
    freshness_sec: int | None = None
    delay_sec: int | None = None
    confidence: float = 0.0
    quality_flags: Mapping[str, Any] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RadarQualityFlags:
    has_market_data: bool
    has_flow_data: bool
    has_greeks_context: bool
    has_oi_context: bool
    provider_quality_ok: bool
    is_degraded: bool
    is_operable: bool
    degradation_reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class RadarSignal:
    symbol: str
    timeframe: RadarTimeframe
    as_of: datetime
    direction_score: float
    volume_confirmation_score: float
    flow_conviction_score: float
    dte_pressure_score: float
    dealer_positioning_proxy_score: float
    aggregate_conviction_score: float
    quality: RadarQualityFlags
    primary_conviction_reason: str
    primary_degradation_reason: str | None = None
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class InterpretedScenario:
    name: str
    conviction: float
    probability: float
    explanation: str
    misinterpretation_risks: tuple[str, ...] = ()
    invalidation_conditions: tuple[str, ...] = ()
    ambiguity_flags: tuple[str, ...] = ()
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RadarSignalBatch:
    symbol: str
    as_of: datetime
    signals: tuple[RadarSignal, ...]
    primary_signal: RadarSignal | None
    scenarios_by_timeframe: Mapping[RadarTimeframe, tuple[InterpretedScenario, ...]] = field(
        default_factory=dict
    )
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RadarDecisionHandoff:
    symbol: str
    as_of: datetime
    operable: bool
    primary_timeframe: RadarTimeframe | None
    primary_signal: RadarSignal | None
    primary_scenarios: tuple[InterpretedScenario, ...] = ()
    signals: tuple[RadarSignal, ...] = ()
    degradation_reasons: tuple[str, ...] = ()
    handoff_summary: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)
