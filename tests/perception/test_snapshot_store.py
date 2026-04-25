from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from atlas_scanner.contracts import InterpretedScenario, RadarQualityFlags, RadarSignal, RadarSignalBatch
from atlas_scanner.store import JsonlRadarSnapshotStore


def _build_signal(symbol: str, timeframe: str, *, operable: bool, degraded_reason: str | None = None) -> RadarSignal:
    quality = RadarQualityFlags(
        has_market_data=True,
        has_flow_data=True,
        has_greeks_context=False,
        has_oi_context=False,
        provider_quality_ok=operable,
        is_degraded=degraded_reason is not None,
        is_operable=operable,
        degradation_reasons=(degraded_reason,) if degraded_reason else (),
    )
    return RadarSignal(
        symbol=symbol,
        timeframe=timeframe,  # type: ignore[arg-type]
        as_of=datetime.now(timezone.utc),
        direction_score=55.0,
        volume_confirmation_score=54.0,
        flow_conviction_score=53.0,
        dte_pressure_score=52.0,
        dealer_positioning_proxy_score=51.0,
        aggregate_conviction_score=53.0,
        quality=quality,
        primary_conviction_reason="direction",
        primary_degradation_reason=degraded_reason,
    )


def test_snapshot_store_append_and_recent(tmp_path: Path) -> None:
    store = JsonlRadarSnapshotStore(tmp_path / "radar.jsonl")
    signal = _build_signal("SPY", "1m", operable=True)
    batch = RadarSignalBatch(
        symbol="SPY",
        as_of=datetime.now(timezone.utc),
        signals=(signal,),
        primary_signal=signal,
        scenarios_by_timeframe={"1m": (InterpretedScenario(name="high_confirmation_breakout_setup", conviction=0.8, probability=0.8, explanation="ok"),)},
    )
    store.append_batch(batch=batch, provider_health={"market:1m": {"last_status": "ok"}})
    rows = store.recent(limit=10)
    assert len(rows) == 1
    assert rows[0].symbol == "SPY"
    assert rows[0].operable is True
    assert rows[0].provider_diagnostics["market:1m"]["last_status"] == "ok"


def test_snapshot_store_replay_filters_degraded(tmp_path: Path) -> None:
    store = JsonlRadarSnapshotStore(tmp_path / "radar.jsonl")
    ok_signal = _build_signal("QQQ", "5m", operable=True)
    bad_signal = _build_signal("QQQ", "5m", operable=False, degraded_reason="missing_market_data")
    batch = RadarSignalBatch(
        symbol="QQQ",
        as_of=datetime.now(timezone.utc),
        signals=(ok_signal, bad_signal),
        primary_signal=ok_signal,
        scenarios_by_timeframe={},
    )
    store.append_batch(batch=batch, provider_health={})
    rows = store.replay(symbol="QQQ", timeframe="5m", limit=10, include_degraded=False)
    assert len(rows) == 1
    assert rows[0].operable is True
    all_rows = store.replay(symbol="QQQ", timeframe="5m", limit=10, include_degraded=True)
    assert len(all_rows) == 2
    assert any(item.degradation_reasons == ("missing_market_data",) for item in all_rows)


def test_snapshot_store_stats_and_rotation(tmp_path: Path) -> None:
    store = JsonlRadarSnapshotStore(tmp_path / "radar.jsonl", max_records=100, max_bytes=100_000)
    signal = _build_signal("IWM", "1m", operable=True)
    batch = RadarSignalBatch(
        symbol="IWM",
        as_of=datetime.now(timezone.utc),
        signals=(signal,),
        primary_signal=signal,
        scenarios_by_timeframe={},
    )
    store.append_batch(batch=batch, provider_health={})
    stats = store.stats()
    assert stats["total_records"] >= 1
    assert stats["file_size_bytes"] > 0
    assert stats["oldest_timestamp"] is not None
    assert stats["newest_timestamp"] is not None
