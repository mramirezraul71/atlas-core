from __future__ import annotations

from atlas_core.brain.models import Command
from atlas_core.brain.workspace_bridge import WorkspaceBridge
from atlas_core.runtime.bootstrap import build_brain_system


def test_workspace_bridge_feedback_states() -> None:
    wb = WorkspaceBridge()
    cmd = Command(target="quant", action="set_mode", params={"mode": "safe"})
    rec = wb.emit_decision(cmd, authorized=False)
    fb = wb.receive_operator_feedback(rec["decision_id"], status="MODIFIED", note="ajusta riesgo")
    assert fb["status"] == "MODIFIED"
    assert fb["approved"] is True


def test_workspace_bridge_timeout_defaults_to_approved() -> None:
    wb = WorkspaceBridge()
    out = wb.receive_operator_feedback(timeout_sec=0.1)
    assert out["ok"] is True
    assert out["approved"] is True
    assert out["status"] == "APPROVED"


def test_workspace_bridge_triggers_healing_cascade_on_anomaly() -> None:
    rt = build_brain_system()
    cmd = Command(target="body", action="speak", params={"text": "hola"})
    evidence = rt.workspace_bridge.analyze_execution_evidence(cmd, {"ok": False, "error": "simulated"})
    assert evidence["anomaly"] is True
    heal = rt.workspace_bridge.trigger_healing_cascade(rt.command_router, reason="test_anomaly", evidence=evidence)
    assert heal["diagnose"]["ok"] is True
    assert heal["recover"]["ok"] is True
