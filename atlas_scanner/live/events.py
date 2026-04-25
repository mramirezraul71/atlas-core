from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Mapping


@dataclass(frozen=True)
class LiveUpdateEvent:
    event_id: str
    event_type: str
    symbol: str
    event_ts: datetime
    ingest_ts: datetime
    source: str
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class EquityTradeLive(LiveUpdateEvent):
    pass


@dataclass(frozen=True)
class EquityQuoteLive(LiveUpdateEvent):
    pass


@dataclass(frozen=True)
class OptionsTradeLive(LiveUpdateEvent):
    pass


@dataclass(frozen=True)
class OptionsQuoteLive(LiveUpdateEvent):
    pass


@dataclass(frozen=True)
class GreeksSnapshotLive(LiveUpdateEvent):
    pass


@dataclass(frozen=True)
class SessionBoundaryEvent(LiveUpdateEvent):
    pass
