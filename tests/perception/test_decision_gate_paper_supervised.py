from __future__ import annotations

import atlas_scanner.api.radar as radar_api


def test_paper_supervised_mode_routes_to_simulated_queue(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_DECISION_GATE_ENABLED", "true")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MODE", "paper_supervised")
    before = len(radar_api._PAPER_SUPERVISED_QUEUE)  # noqa: SLF001
    result = radar_api.build_realtime_snapshot(symbol="SPY", timeframes=("1m",), runtime_mode="paper")
    assert result.handoff.metadata.get("decision_gate") is not None
    after = len(radar_api._PAPER_SUPERVISED_QUEUE)  # noqa: SLF001
    assert after >= before + 1
