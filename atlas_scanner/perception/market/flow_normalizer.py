from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from typing import Any, Iterable, Literal, Mapping

from atlas_scanner.contracts import FlowPerceptionSnapshot, RadarTimeframe

FlowType = Literal["call", "put", "equity"]
AggressionType = Literal["passive", "neutral", "aggressive"]
EventType = Literal["sweep", "block", "regular"]

_DTE_BUCKETS: tuple[tuple[str, int, int], ...] = (
    ("0-2", 0, 2),
    ("3-7", 3, 7),
    ("8-30", 8, 30),
    ("31-90", 31, 90),
    ("90+", 91, 100000),
)
_WINDOWS: dict[RadarTimeframe, timedelta] = {
    "1m": timedelta(minutes=1),
    "5m": timedelta(minutes=5),
    "15m": timedelta(minutes=15),
    "1h": timedelta(hours=1),
    "1d": timedelta(days=1),
}


@dataclass(frozen=True)
class RawFlowEvent:
    symbol: str
    event_ts: datetime
    side: Literal["buy", "sell"]
    size: float
    strike: float | None
    dte: int | None
    premium: float
    aggression: AggressionType = "neutral"
    type: FlowType = "equity"
    event_kind: EventType = "regular"
    meta: Mapping[str, Any] = field(default_factory=dict)


def synthetic_flow_events(symbol: str, *, now: datetime | None = None) -> tuple[RawFlowEvent, ...]:
    ts = now or datetime.now(timezone.utc)
    return (
        RawFlowEvent(symbol=symbol, event_ts=ts - timedelta(seconds=15), side="buy", size=1200, strike=450.0, dte=2, premium=240000.0, aggression="aggressive", type="call", event_kind="sweep"),
        RawFlowEvent(symbol=symbol, event_ts=ts - timedelta(seconds=40), side="buy", size=800, strike=430.0, dte=7, premium=160000.0, aggression="aggressive", type="put", event_kind="block"),
        RawFlowEvent(symbol=symbol, event_ts=ts - timedelta(seconds=50), side="buy", size=5000, strike=None, dte=None, premium=0.0, aggression="neutral", type="equity"),
    )


def normalize_flow_events(
    *,
    symbol: str,
    timeframe: RadarTimeframe,
    events: Iterable[RawFlowEvent],
    as_of: datetime | None = None,
    diagnostics: Mapping[str, str] | None = None,
) -> FlowPerceptionSnapshot:
    now = as_of or datetime.now(timezone.utc)
    window = _WINDOWS[timeframe]
    relevant = [
        event
        for event in events
        if event.symbol.upper() == symbol.upper() and event.event_ts >= (now - window)
    ]
    call_volume = sum(event.size for event in relevant if event.type == "call")
    put_volume = sum(event.size for event in relevant if event.type == "put")
    call_premium = sum(event.premium for event in relevant if event.type == "call")
    put_premium = sum(event.premium for event in relevant if event.type == "put")
    equity_buy = sum(event.size for event in relevant if event.type == "equity" and event.side == "buy")
    equity_sell = sum(event.size for event in relevant if event.type == "equity" and event.side == "sell")
    equity_total = equity_buy + equity_sell
    equity_flow_bias = ((equity_buy - equity_sell) / equity_total) if equity_total > 0 else None
    total_premium = call_premium + put_premium
    net_bias = ((call_premium - put_premium) / total_premium) if total_premium > 0 else 0.0
    aggression_score = _aggression_score(relevant)
    dte_call, dte_put = _dte_premium_maps(relevant)
    dte_distribution = _dte_distribution(dte_call=dte_call, dte_put=dte_put)
    quality_flags = {
        "has_flow": len(relevant) > 0,
        "has_options_flow": (call_volume + put_volume) > 0,
        "sweep_detected": any(event.event_kind == "sweep" for event in relevant),
        "block_detected": any(event.event_kind == "block" for event in relevant),
        "events_count": len(relevant),
        "window_sec": int(window.total_seconds()),
    }
    return FlowPerceptionSnapshot(
        symbol=symbol.upper(),
        timeframe=timeframe,
        as_of=now,
        equity_flow_bias=equity_flow_bias,
        call_volume=call_volume or None,
        put_volume=put_volume or None,
        call_premium=call_premium or None,
        put_premium=put_premium or None,
        net_bias=net_bias,
        aggression_score=aggression_score,
        dte_distribution=dte_distribution,
        dte_call_premium=dte_call,
        dte_put_premium=dte_put,
        quality_flags=quality_flags,
        diagnostics=dict(diagnostics or {}),
        meta={"synthetic": False},
    )


def _aggression_score(events: list[RawFlowEvent]) -> float:
    if not events:
        return 0.0
    values = {"passive": 0.2, "neutral": 0.5, "aggressive": 1.0}
    return sum(values.get(event.aggression, 0.5) for event in events) / len(events)


def _dte_bucket(dte: int | None) -> str:
    if dte is None:
        return "unknown"
    for label, lo, hi in _DTE_BUCKETS:
        if lo <= dte <= hi:
            return label
    return "unknown"


def _dte_premium_maps(events: list[RawFlowEvent]) -> tuple[dict[str, float], dict[str, float]]:
    call: dict[str, float] = {}
    put: dict[str, float] = {}
    for event in events:
        bucket = _dte_bucket(event.dte)
        if event.type == "call":
            call[bucket] = call.get(bucket, 0.0) + event.premium
        elif event.type == "put":
            put[bucket] = put.get(bucket, 0.0) + event.premium
    return call, put


def _dte_distribution(*, dte_call: Mapping[str, float], dte_put: Mapping[str, float]) -> dict[str, float]:
    merged: dict[str, float] = {}
    for key, value in dte_call.items():
        merged[key] = merged.get(key, 0.0) + value
    for key, value in dte_put.items():
        merged[key] = merged.get(key, 0.0) + value
    total = sum(merged.values())
    if total <= 0:
        return {}
    return {key: value / total for key, value in merged.items()}
