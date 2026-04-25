from __future__ import annotations

from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from typing import Sequence

from atlas_scanner.live.config import LiveScannerConfig
from atlas_scanner.live.events import LiveUpdateEvent
from atlas_scanner.live.interfaces import (
    IncrementalScoringEngine,
    LiveScannerService,
    LiveServiceHealth,
    LiveStateStore,
)
from atlas_scanner.live.state import FlowWindowState, LiveSymbolSnapshot, LiveSymbolState


@dataclass
class InMemoryLiveStateStore(LiveStateStore):
    _states: dict[str, LiveSymbolState] = field(default_factory=dict)

    def upsert(self, state: LiveSymbolState) -> None:
        self._states[state.symbol.upper()] = state

    def get(self, symbol: str) -> LiveSymbolState | None:
        return self._states.get(symbol.upper())

    def list_top(self, limit: int) -> tuple[LiveSymbolState, ...]:
        ranked = sorted(
            self._states.values(),
            key=lambda item: item.multifactor_score,
            reverse=True,
        )
        return tuple(ranked[: max(0, limit)])

    def mark_stale(self, now: datetime) -> int:
        _ = now
        # No-op stale handling in skeleton phase.
        return 0


@dataclass
class NoOpIncrementalScoringEngine(IncrementalScoringEngine):
    def apply_event(
        self,
        prev: LiveSymbolState | None,
        event: LiveUpdateEvent,
        config: LiveScannerConfig,
    ) -> LiveSymbolState:
        _ = config
        if prev is not None:
            return replace(prev, last_update_ts=event.event_ts, is_stale=False)
        return LiveSymbolState(
            symbol=event.symbol.upper(),
            last_update_ts=event.event_ts,
            is_stale=False,
            market_state="open",
            equity_last=None,
            equity_quote=None,
            options_flow_window=FlowWindowState(),
            greeks_state=None,
            institutional_flow_score=0.0,
            multifactor_score=0.0,
        )

    def recompute_scores(
        self,
        state: LiveSymbolState,
        config: LiveScannerConfig,
    ) -> LiveSymbolState:
        _ = config
        return state


@dataclass
class InMemoryLiveScannerService(LiveScannerService):
    config: LiveScannerConfig
    state_store: LiveStateStore
    scoring_engine: IncrementalScoringEngine
    _running: bool = False

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def ingest(self, event: LiveUpdateEvent) -> None:
        previous = self.state_store.get(event.symbol)
        updated = self.scoring_engine.apply_event(
            prev=previous,
            event=event,
            config=self.config,
        )
        rescored = self.scoring_engine.recompute_scores(state=updated, config=self.config)
        self.state_store.upsert(rescored)

    def ingest_batch(self, events: Sequence[LiveUpdateEvent]) -> None:
        for event in events:
            self.ingest(event)

    def get_symbol_snapshot(self, symbol: str) -> LiveSymbolSnapshot | None:
        state = self.state_store.get(symbol)
        if state is None:
            return None
        as_of = state.last_update_ts or datetime.now(timezone.utc)
        return LiveSymbolSnapshot(
            symbol=state.symbol,
            as_of=as_of,
            multifactor_score=state.multifactor_score,
            institutional_flow_score=state.institutional_flow_score,
            component_scores=state.component_scores,
            rank=None,
            is_stale=state.is_stale,
            lags_ms=None,
            flags=state.flags,
            explain=state.explain,
        )

    def get_top_snapshot(self, limit: int) -> tuple[LiveSymbolSnapshot, ...]:
        states = self.state_store.list_top(limit)
        snapshots: list[LiveSymbolSnapshot] = []
        for rank, state in enumerate(states, start=1):
            as_of = state.last_update_ts or datetime.now(timezone.utc)
            snapshots.append(
                LiveSymbolSnapshot(
                    symbol=state.symbol,
                    as_of=as_of,
                    multifactor_score=state.multifactor_score,
                    institutional_flow_score=state.institutional_flow_score,
                    component_scores=state.component_scores,
                    rank=rank,
                    is_stale=state.is_stale,
                    lags_ms=None,
                    flags=state.flags,
                    explain=state.explain,
                )
            )
        return tuple(snapshots)

    def health(self) -> LiveServiceHealth:
        status = "running" if self._running else "stopped"
        num_symbols = len(self.state_store.list_top(limit=1_000_000))
        return LiveServiceHealth(
            status=status,
            lag_ms=None,
            num_symbols=num_symbols,
            meta={},
        )
