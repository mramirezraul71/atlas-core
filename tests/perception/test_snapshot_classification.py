from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.inference.radar_service import RadarRunInput, run_radar_first_cut


def test_snapshot_classification_is_exposed_in_signal_meta() -> None:
    now = datetime.now(timezone.utc)
    batch = run_radar_first_cut(
        RadarRunInput(
            symbol="SPY",
            market_by_timeframe={"1m": {"spot_price": 500.0, "return_pct": 0.2, "momentum_score": 55.0}},
            flow_by_timeframe={"1m": {"call_premium": 100000.0, "put_premium": 80000.0}},
            as_of=now,
            timeframes=("1m",),
        )
    )
    assert batch.signals
    classification = batch.signals[0].meta.get("snapshot_classification")
    assert classification in {
        "fully_operable",
        "operable_with_degradation",
        "structural_only",
        "fast_only",
        "non_operable",
    }
