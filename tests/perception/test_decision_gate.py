from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarDecisionHandoff, RadarQualityFlags, RadarSignal
from atlas_scanner.decision.gate import DecisionGateConfig, evaluate_handoff


def _signal(classification: str, *, structural: float = 60.0, fast: float = 60.0, divergence: float = 10.0) -> RadarSignal:
    return RadarSignal(
        symbol="SPY",
        timeframe="5m",
        as_of=datetime.now(timezone.utc),
        direction_score=55.0,
        volume_confirmation_score=55.0,
        flow_conviction_score=55.0,
        dte_pressure_score=55.0,
        dealer_positioning_proxy_score=55.0,
        aggregate_conviction_score=58.0,
        quality=RadarQualityFlags(
            has_market_data=True,
            has_flow_data=True,
            has_greeks_context=True,
            has_oi_context=True,
            provider_quality_ok=True,
            is_degraded=classification == "operable_with_degradation",
            is_operable=classification != "non_operable",
            degradation_reasons=("degraded",) if classification == "operable_with_degradation" else (),
        ),
        primary_conviction_reason="test",
        meta={
            "snapshot_classification": classification,
            "structural_confidence_score": structural,
            "fast_pressure_score": fast,
            "fast_structural_divergence_score": divergence,
            "fast_structural_alignment": "aligned",
            "horizon_conflict": False,
            "cross_horizon_alignment": True,
        },
    )


def _handoff(signal: RadarSignal) -> RadarDecisionHandoff:
    return RadarDecisionHandoff(
        symbol=signal.symbol,
        as_of=signal.as_of,
        operable=signal.quality.is_operable,
        primary_timeframe=signal.timeframe,
        primary_signal=signal,
        primary_scenarios=(),
        signals=(signal,),
        degradation_reasons=signal.quality.degradation_reasons,
        handoff_summary="test",
        metadata={},
    )


def test_decision_gate_accepts_fully_operable() -> None:
    cfg = DecisionGateConfig(enabled=True)
    eval_result = evaluate_handoff(_handoff(_signal("fully_operable")), cfg)
    assert eval_result.decision == "accepted"


def test_decision_gate_caution_for_degraded_when_allowed() -> None:
    cfg = DecisionGateConfig(enabled=True, allow_degraded=True)
    eval_result = evaluate_handoff(_handoff(_signal("operable_with_degradation")), cfg)
    assert eval_result.decision == "caution"


def test_decision_gate_rejects_non_operable() -> None:
    cfg = DecisionGateConfig(enabled=True)
    eval_result = evaluate_handoff(_handoff(_signal("non_operable")), cfg)
    assert eval_result.decision == "rejected"


def test_decision_gate_respects_structural_only_flag() -> None:
    cfg = DecisionGateConfig(enabled=True, allow_structural_only=False)
    eval_result = evaluate_handoff(_handoff(_signal("structural_only")), cfg)
    assert eval_result.decision == "rejected"


def test_decision_gate_respects_fast_only_flag() -> None:
    cfg = DecisionGateConfig(enabled=True, allow_fast_only=False)
    eval_result = evaluate_handoff(_handoff(_signal("fast_only")), cfg)
    assert eval_result.decision == "rejected"
