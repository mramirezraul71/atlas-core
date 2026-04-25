from __future__ import annotations

from atlas_scanner.api.radar import build_realtime_snapshot


def test_realtime_snapshot_includes_political_freshness_and_provider() -> None:
    result = build_realtime_snapshot(
        symbol="SPY",
        timeframes=("1m", "5m"),
        runtime_mode="paper",
    )
    assert result.batch.signals
    first = result.batch.signals[0]
    freshness = first.meta.get("freshness", {})
    assert "political" in freshness.get("intraday", {})
    assert any(key.startswith("political:") for key in (first.meta.get("provider_status", {}) or {}).keys())
