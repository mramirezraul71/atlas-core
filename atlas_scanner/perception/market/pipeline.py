from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import FlowPerceptionSnapshot, MarketPerceptionSnapshot, RadarTimeframe


@dataclass(frozen=True)
class MarketPerceptionInput:
    symbol: str
    timeframe: RadarTimeframe
    market_data: Mapping[str, Any]
    flow_data: Mapping[str, Any]
    provider_diagnostics: Mapping[str, str]
    as_of: datetime | None = None


def _optional_float(raw: object) -> float | None:
    if isinstance(raw, (int, float)):
        return float(raw)
    return None


def build_market_perception(input_data: MarketPerceptionInput) -> MarketPerceptionSnapshot:
    as_of = input_data.as_of or datetime.now(timezone.utc)
    md = dict(input_data.market_data)
    return MarketPerceptionSnapshot(
        symbol=input_data.symbol.upper(),
        timeframe=input_data.timeframe,
        as_of=as_of,
        spot_price=_optional_float(md.get("spot_price")),
        close_price=_optional_float(md.get("close_price")),
        return_pct=_optional_float(md.get("return_pct")),
        momentum_score=_optional_float(md.get("momentum_score")),
        relative_volume=_optional_float(md.get("relative_volume")),
        volume_acceleration=_optional_float(md.get("volume_acceleration")),
        diagnostics=dict(input_data.provider_diagnostics),
        meta=md.get("meta") if isinstance(md.get("meta"), Mapping) else {},
    )


def build_flow_perception(input_data: MarketPerceptionInput) -> FlowPerceptionSnapshot:
    as_of = input_data.as_of or datetime.now(timezone.utc)
    fd = dict(input_data.flow_data)
    dte_call = fd.get("dte_call_premium")
    dte_put = fd.get("dte_put_premium")
    return FlowPerceptionSnapshot(
        symbol=input_data.symbol.upper(),
        timeframe=input_data.timeframe,
        as_of=as_of,
        equity_flow_bias=_optional_float(fd.get("equity_flow_bias")),
        call_volume=_optional_float(fd.get("call_volume")),
        put_volume=_optional_float(fd.get("put_volume")),
        call_premium=_optional_float(fd.get("call_premium")),
        put_premium=_optional_float(fd.get("put_premium")),
        net_bias=_optional_float(fd.get("net_bias")),
        aggression_score=_optional_float(fd.get("aggression_score")),
        dte_distribution=dict(fd.get("dte_distribution")) if isinstance(fd.get("dte_distribution"), Mapping) else {},
        dte_call_premium=dict(dte_call) if isinstance(dte_call, Mapping) else {},
        dte_put_premium=dict(dte_put) if isinstance(dte_put, Mapping) else {},
        call_moneyness_distance_pct=_optional_float(fd.get("call_moneyness_distance_pct")),
        put_moneyness_distance_pct=_optional_float(fd.get("put_moneyness_distance_pct")),
        post_flow_price_reaction_pct=_optional_float(fd.get("post_flow_price_reaction_pct")),
        net_gamma=_optional_float(fd.get("net_gamma")),
        oi_concentration=_optional_float(fd.get("oi_concentration")),
        iv_context=_optional_float(fd.get("iv_context")),
        quality_flags=dict(fd.get("quality_flags")) if isinstance(fd.get("quality_flags"), Mapping) else {},
        diagnostics=dict(input_data.provider_diagnostics),
        meta=fd.get("meta") if isinstance(fd.get("meta"), Mapping) else {},
    )
