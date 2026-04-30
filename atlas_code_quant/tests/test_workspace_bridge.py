from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import Command, Event
from atlas_core.brain.workspace_bridge import WorkspaceBridge
from atlas_core.runtime.bootstrap import build_brain_system


def test_workspace_bridge_emit_feedback_and_evidence() -> None:
    wb = WorkspaceBridge()
    cmd = Command(target="quant", action="set_mode", params={"mode": "safe"})
    emitted = wb.emit_decision(cmd, authorized=False, stage="pre_authorization")
    feedback = wb.receive_operator_feedback(emitted["decision_id"], approved=True, note="ok")
    waited = wb.wait_for_operator_feedback(emitted["decision_id"], timeout_sec=0.3, poll_sec=0.01)

    assert feedback["approved"] is True
    assert waited["ok"] is True
    assert waited["approved"] is True

    anomaly = wb.analyze_execution_evidence(cmd, {"ok": False, "error": "forced_fail"})
    assert anomaly["anomaly"] is True
    assert "execution_not_ok" in anomaly["reasons"]


def test_braincore_supervised_requires_operator_approval(tmp_path: Path) -> None:
    class AutoRejectWorkspace(WorkspaceBridge):
        def wait_for_operator_feedback(self, decision_id: str, timeout_sec: float = 30.0, poll_sec: float = 0.1) -> dict:
            return {"ok": True, "decision_id": decision_id, "approved": False, "note": "reject"}

    runtime = build_brain_system(operation_center_path=tmp_path / "operation_center_state.json")
    runtime.workspace_bridge = AutoRejectWorkspace(audit_log=runtime.audit_log)
    runtime.brain._workspace = runtime.workspace_bridge  # type: ignore[attr-defined]

    runtime.brain.set_mode("supervised")
    runtime.brain.ingest_event(Event(source="operator", kind="speak", payload={"text": "hola"}))
    out = runtime.brain.evaluate()
    assert out
    assert out[0]["ok"] is False
    assert out[0]["error"] == "operator_rejected"
