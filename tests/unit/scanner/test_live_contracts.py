from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.live import (
    EquityTradeLive,
    InMemoryLiveScannerService,
    InMemoryLiveStateStore,
    LiveScannerConfig,
    NoOpIncrementalScoringEngine,
)


def test_live_types_can_be_instantiated() -> None:
    config = LiveScannerConfig(symbols=("SPY", "QQQ"), top_k=5)
    assert config.symbols == ("SPY", "QQQ")
    assert config.top_k == 5

    event = EquityTradeLive(
        event_id="evt-1",
        event_type="equity_trade",
        symbol="SPY",
        event_ts=datetime(2026, 1, 1, 14, 30, tzinfo=timezone.utc),
        ingest_ts=datetime(2026, 1, 1, 14, 30, 1, tzinfo=timezone.utc),
        source="test-feed",
        payload={"price": 601.25},
    )
    assert event.symbol == "SPY"
    assert event.payload["price"] == 601.25


def test_inmemory_live_service_stub_ingests_and_returns_snapshots() -> None:
    service = InMemoryLiveScannerService(
        config=LiveScannerConfig(symbols=("SPY",), top_k=10),
        state_store=InMemoryLiveStateStore(),
        scoring_engine=NoOpIncrementalScoringEngine(),
    )
    service.start()
    service.ingest(
        EquityTradeLive(
            event_id="evt-1",
            event_type="equity_trade",
            symbol="SPY",
            event_ts=datetime(2026, 1, 1, 14, 30, tzinfo=timezone.utc),
            ingest_ts=datetime(2026, 1, 1, 14, 30, 1, tzinfo=timezone.utc),
            source="test-feed",
            payload={},
        )
    )

    symbol_snapshot = service.get_symbol_snapshot("SPY")
    assert symbol_snapshot is not None
    assert symbol_snapshot.symbol == "SPY"

    top = service.get_top_snapshot(limit=5)
    assert len(top) == 1
    assert top[0].rank == 1

    health = service.health()
    assert health.status == "running"
    assert health.num_symbols == 1

    service.stop()
    assert service.health().status == "stopped"
