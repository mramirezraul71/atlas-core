from atlas_code_quant.vision.timing_gate import GateInput, VisionTimingGate


def test_vision_gate_stub_degraded_allow():
    gate = VisionTimingGate()
    out = gate.evaluate(GateInput(symbol="SPY", requires_visual_confirmation=False))
    assert out.decision == "allow"
    assert out.degraded is True


def test_vision_gate_stub_blocks_when_visual_required():
    gate = VisionTimingGate()
    out = gate.evaluate(GateInput(symbol="SPY", requires_visual_confirmation=True))
    assert out.decision == "block"
    assert out.degraded is True
