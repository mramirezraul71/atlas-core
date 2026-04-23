from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Mapping, Protocol, Sequence

from atlas_scanner.live.config import LiveScannerConfig
from atlas_scanner.live.events import LiveUpdateEvent
from atlas_scanner.live.state import LiveSymbolSnapshot, LiveSymbolState


@dataclass(frozen=True)
class FeedHealth:
    status: str
    last_heartbeat: datetime | None = None
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class LiveServiceHealth:
    status: str
    lag_ms: int | None = None
    num_symbols: int = 0
    meta: Mapping[str, Any] = field(default_factory=dict)


class LiveDataFeed(Protocol):
    def connect(self) -> None:
        ...

    def subscribe(self, symbols: Sequence[str]) -> None:
        ...

    def poll(self, batch_size: int) -> Sequence[LiveUpdateEvent]:
        ...

    def health(self) -> FeedHealth:
        ...

    def close(self) -> None:
        ...


class LiveStateStore(Protocol):
    def upsert(self, state: LiveSymbolState) -> None:
        ...

    def get(self, symbol: str) -> LiveSymbolState | None:
        ...

    def list_top(self, limit: int) -> Sequence[LiveSymbolState]:
        ...

    def mark_stale(self, now: datetime) -> int:
        ...


class IncrementalScoringEngine(Protocol):
    def apply_event(
        self,
        prev: LiveSymbolState | None,
        event: LiveUpdateEvent,
        config: LiveScannerConfig,
    ) -> LiveSymbolState:
        ...

    def recompute_scores(
        self,
        state: LiveSymbolState,
        config: LiveScannerConfig,
    ) -> LiveSymbolState:
        ...


class LiveScannerService(Protocol):
    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def ingest(self, event: LiveUpdateEvent) -> None:
        ...

    def ingest_batch(self, events: Sequence[LiveUpdateEvent]) -> None:
        ...

    def get_symbol_snapshot(self, symbol: str) -> LiveSymbolSnapshot | None:
        ...

    def get_top_snapshot(self, limit: int) -> Sequence[LiveSymbolSnapshot]:
        ...

    def health(self) -> LiveServiceHealth:
        ...
