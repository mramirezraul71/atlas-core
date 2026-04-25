from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Literal, Mapping


@dataclass(frozen=True)
class QuoteState:
    bid: float | None = None
    ask: float | None = None
    mid: float | None = None
    last_quote_ts: datetime | None = None


@dataclass(frozen=True)
class FlowWindowState:
    window_seconds: int = 300
    call_volume: float = 0.0
    put_volume: float = 0.0
    net_premium: float = 0.0
    last_event_ts: datetime | None = None


@dataclass(frozen=True)
class GreeksState:
    net_gamma: float | None = None
    net_delta: float | None = None
    iv_rank: float | None = None
    last_greeks_ts: datetime | None = None


@dataclass(frozen=True)
class LiveSymbolState:
    symbol: str
    last_update_ts: datetime | None
    is_stale: bool
    market_state: Literal["pre", "open", "post", "closed"]
    equity_last: float | None
    equity_quote: QuoteState | None
    options_flow_window: FlowWindowState
    greeks_state: GreeksState | None
    institutional_flow_score: float
    multifactor_score: float
    component_scores: Mapping[str, float] = field(default_factory=dict)
    flags: tuple[str, ...] = ()
    explain: tuple[str, ...] = ()
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class LiveSymbolSnapshot:
    symbol: str
    as_of: datetime
    multifactor_score: float
    institutional_flow_score: float
    component_scores: Mapping[str, float] = field(default_factory=dict)
    rank: int | None = None
    is_stale: bool = True
    lags_ms: int | None = None
    flags: tuple[str, ...] = ()
    explain: tuple[str, ...] = ()
