from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from atlas_scanner.decision.gate import DecisionGateEvaluation
from atlas_scanner.decision.store import JsonlDecisionGateStore


def _eval(symbol: str = "SPY", decision: str = "accepted") -> DecisionGateEvaluation:
    return DecisionGateEvaluation(
        timestamp=datetime.now(timezone.utc),
        symbol=symbol,
        timeframe="5m",
        snapshot_classification="fully_operable",
        structural_confidence_score=62.0,
        fast_pressure_score=58.0,
        fast_structural_alignment="aligned",
        fast_structural_divergence_score=9.0,
        horizon_conflict=False,
        cross_horizon_alignment=True,
        decision=decision,  # type: ignore[arg-type]
        reason="gate_passed",
        thresholds={"min_structural_confidence": 45},
        mode="paper_supervised",
    )


def test_decision_store_append_recent_and_replay(tmp_path: Path) -> None:
    store = JsonlDecisionGateStore(tmp_path / "decision_gate.jsonl")
    store.append(_eval(symbol="SPY", decision="accepted"))
    store.append(_eval(symbol="QQQ", decision="rejected"))
    recent = store.recent(limit=10)
    assert len(recent) == 2
    replay_spy = store.replay(symbol="SPY", limit=10)
    assert len(replay_spy) == 1
    assert replay_spy[0].symbol == "SPY"


def test_decision_store_rotation_and_stats(tmp_path: Path) -> None:
    store = JsonlDecisionGateStore(tmp_path / "decision_gate.jsonl", max_records=100, max_bytes=100_000)
    for _ in range(120):
        store.append(_eval(symbol="SPY", decision="accepted"))
    stats = store.stats()
    assert stats["record_count"] > 0
    assert stats["total_decisions"] > 0
    assert stats["accepted_count"] > 0
    assert stats["file_size_bytes"] > 0
