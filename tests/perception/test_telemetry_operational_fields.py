from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.inference.radar_service import RadarRunInput, run_radar_first_cut
from atlas_scanner.telemetry.recorder import InMemoryRadarTelemetryRecorder


def test_telemetry_contains_operational_composites() -> None:
    now = datetime.now(timezone.utc)
    batch = run_radar_first_cut(
        RadarRunInput(
            symbol="SPY",
            market_by_timeframe={"1m": {"spot_price": 500.0, "return_pct": 0.2, "momentum_score": 57.0}},
            flow_by_timeframe={"1m": {"call_premium": 110000.0, "put_premium": 90000.0}},
            as_of=now,
            timeframes=("1m",),
        )
    )
    recorder = InMemoryRadarTelemetryRecorder()
    events = recorder.record_batch(batch, pipeline="test", sources_used=("market:1m",))
    assert events
    meta = events[0].meta
    assert "structural_confidence_score" in meta
    assert "fast_pressure_score" in meta
    assert "fast_structural_alignment" in meta
    assert "snapshot_classification" in meta
