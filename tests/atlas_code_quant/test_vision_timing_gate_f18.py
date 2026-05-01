"""Tests F18 — VisionTimingGate (paper-only).

Cubre:

* Camera OK → ALLOW.
* Camera unavailable + requires_visual_confirmation=False → ALLOW degraded.
* Camera unavailable + requires_visual_confirmation=True → BLOCK degraded.
* Captura que lanza → degraded + camera_status reflejando error.
* Pattern evaluator inyectable: DELAY y FORCE_EXIT.
* Pattern evaluator que lanza → BLOCK defensivo.
* Pattern evaluator con retorno inválido → BLOCK defensivo.
* GateInput None → BLOCK defensivo.
"""

from __future__ import annotations

import pytest

from atlas_code_quant.vision.timing_gate import (
    GateInput,
    GateOutput,
    VisionDecision,
    VisionTimingGate,
)


def _probe_ok():
    return {"ok": True, "source": "usb_dshow", "error": None}


def _probe_unavailable():
    return {"ok": False, "source": "none", "error": "camera_disconnected"}


def _probe_raises():
    raise RuntimeError("simulated probe boom")


class TestPolicyDefault:
    def test_camera_ok_allow(self):
        g = VisionTimingGate(capture_probe=_probe_ok)
        out = g.evaluate(GateInput(strategy_id="vs1"))
        assert out.decision == VisionDecision.ALLOW
        assert out.degraded is False
        assert out.camera_status == "ok"

    def test_unavailable_optional_allow_degraded(self):
        g = VisionTimingGate(capture_probe=_probe_unavailable)
        out = g.evaluate(
            GateInput(strategy_id="vs1", requires_visual_confirmation=False)
        )
        assert out.decision == VisionDecision.ALLOW
        assert out.degraded is True
        assert out.camera_status == "unavailable"

    def test_unavailable_required_block_degraded(self):
        g = VisionTimingGate(capture_probe=_probe_unavailable)
        out = g.evaluate(
            GateInput(strategy_id="vs1", requires_visual_confirmation=True)
        )
        assert out.decision == VisionDecision.BLOCK
        assert out.degraded is True
        assert out.camera_status == "unavailable"

    def test_probe_raises_block_or_allow_degraded(self):
        # probe que lanza: el wrapper trata como cap inválida + error
        g = VisionTimingGate(capture_probe=_probe_raises)
        out = g.evaluate(
            GateInput(strategy_id="vs1", requires_visual_confirmation=True)
        )
        # required + cam.ok=False → BLOCK
        assert out.decision == VisionDecision.BLOCK
        assert out.degraded is True


class TestInjectedEvaluator:
    def test_delay_decision(self):
        def ev(inp, cap):
            return GateOutput(
                decision=VisionDecision.DELAY,
                reason="awaiting_pattern_confirmation",
                degraded=False,
                camera_status="ok",
            )

        g = VisionTimingGate(capture_probe=_probe_ok, pattern_evaluator=ev)
        out = g.evaluate(GateInput())
        assert out.decision == VisionDecision.DELAY
        assert "awaiting_pattern" in out.reason

    def test_force_exit_decision(self):
        def ev(inp, cap):
            return GateOutput(
                decision=VisionDecision.FORCE_EXIT,
                reason="reversal_pattern_detected",
                degraded=False,
                camera_status="ok",
            )

        g = VisionTimingGate(capture_probe=_probe_ok, pattern_evaluator=ev)
        out = g.evaluate(GateInput())
        assert out.decision == VisionDecision.FORCE_EXIT

    def test_evaluator_raises_block(self):
        def ev(inp, cap):
            raise ValueError("pattern engine boom")

        g = VisionTimingGate(capture_probe=_probe_ok, pattern_evaluator=ev)
        out = g.evaluate(GateInput())
        assert out.decision == VisionDecision.BLOCK
        assert "pattern_evaluator_raised" in out.reason
        assert out.degraded is True

    def test_evaluator_invalid_return(self):
        def ev(inp, cap):
            return "not a GateOutput"  # type: ignore[return-value]

        g = VisionTimingGate(capture_probe=_probe_ok, pattern_evaluator=ev)
        out = g.evaluate(GateInput())
        assert out.decision == VisionDecision.BLOCK
        assert "invalid_return" in out.reason


class TestDefensiveInputs:
    def test_none_input(self):
        g = VisionTimingGate()
        out = g.evaluate(None)  # type: ignore[arg-type]
        assert out.decision == VisionDecision.BLOCK
        assert out.degraded is True

    def test_default_no_probe_means_unavailable(self):
        # sin inyectar probe: default devuelve ok=False
        g = VisionTimingGate()
        out = g.evaluate(
            GateInput(strategy_id="vs1", requires_visual_confirmation=False)
        )
        assert out.decision == VisionDecision.ALLOW
        assert out.degraded is True
