"""Tests F7 — VisionTimingGate.evaluate()."""
from __future__ import annotations

from atlas_code_quant.vision.imbalance import from_score
from atlas_code_quant.vision.timing_gate import (
    GateInput,
    VisionTimingGate,
)


def test_block_when_camera_unavailable_and_visual_required() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            requires_visual_confirmation=True,
            camera_state="CAMERA_UNAVAILABLE",
        )
    )
    assert out.decision == "block"
    assert out.degraded is True
    assert "camera_unavailable" in out.reason


def test_allow_degraded_when_camera_unavailable_no_visual_required() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            requires_visual_confirmation=False,
            camera_state="CAMERA_UNAVAILABLE",
        )
    )
    assert out.decision == "allow"
    assert out.degraded is True
    assert 0.0 < out.confidence < 1.0


def test_exit_intent_always_allowed() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="exit",
            requires_visual_confirmation=True,
            camera_state="CAMERA_UNAVAILABLE",
        )
    )
    assert out.decision == "allow"
    assert out.reason == "exit_intent_bypass"


def test_camera_ok_confirmation_high_confidence() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            camera_state="CAMERA_OK",
            pattern="confirmation",
        )
    )
    assert out.decision == "allow"
    assert out.degraded is False
    assert out.confidence >= 0.80


def test_camera_ok_noise_delays() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            camera_state="CAMERA_OK",
            pattern="noise",
        )
    )
    assert out.decision == "delay"


def test_imbalance_side_propagated() -> None:
    out = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            camera_state="CAMERA_OK",
            pattern="setup",
            imbalance=from_score(0.5),
        )
    )
    assert out.imbalance_side == "buy_pressure"

    out_neg = VisionTimingGate().evaluate(
        GateInput(
            symbol="AAPL",
            intent="entry",
            camera_state="CAMERA_OK",
            pattern="setup",
            imbalance=from_score(-0.5),
        )
    )
    assert out_neg.imbalance_side == "sell_pressure"
