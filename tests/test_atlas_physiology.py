from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from atlas_core.brain.models import Command, Event
from atlas_core.brain.physiological_dashboard import PhysiologicalDashboard
from atlas_core.runtime.bootstrap import build_brain_system
from atlas_core.runtime.event_loop import start_event_loop_background
from atlas_core.runtime.heartbeat_loop import start_heartbeat_background


def _run_short_runtime(tmp_path: Path, seconds: float = 0.8):
    op_state = tmp_path / "operation_center_state.json"
    op_state.write_text("{}", encoding="utf-8")
    rt = build_brain_system(operation_center_path=op_state)
    audit_path = tmp_path / "brain_audit_log.jsonl"
    rt.audit_log.path = audit_path
    stop = threading.Event()
    hb = start_heartbeat_background(rt.adapters, rt.state_bus, stop, interval_sec=0.1)
    ev = start_event_loop_background(rt.brain, stop, interval_sec=0.05)
    rt.brain.ingest_event(Event(source="operator", kind="speak", payload={"text": "test"}))
    rt.brain.ingest_event(Event(source="quant", kind="risk_alert", payload={"mode": "safe"}))
    time.sleep(seconds)
    stop.set()
    hb.join(timeout=2.0)
    ev.join(timeout=2.0)
    dashboard = PhysiologicalDashboard(
        state_bus=rt.state_bus,
        workspace_bridge=rt.workspace_bridge,
        audit_log_path=audit_path,
        learning_root=Path(r"c:\ATLAS_PUSH\atlas_code_quant\data\learning"),
    )
    return rt, dashboard.snapshot(), audit_path


def test_heartbeat_regularity(tmp_path: Path) -> None:
    _, payload, _ = _run_short_runtime(tmp_path)
    assert payload["heartbeat"]["bpm"] > 0
    assert payload["heartbeat"]["rhythm"] in {"regular", "irregular", "critical"}


def test_brain_response_time(tmp_path: Path) -> None:
    rt, payload, _ = _run_short_runtime(tmp_path, seconds=0.5)
    for _ in range(100):
        rt.brain.evaluate_and_decide()
    assert payload["nervous_system"]["latency_brain_ms"] < 500


def test_safety_kernel_blocks(tmp_path: Path) -> None:
    rt, _, _ = _run_short_runtime(tmp_path)
    rt.brain.set_mode("safe")
    allowed, reason = rt.safety_kernel.authorize(
        Command(target="quant", action="submit", params={}),
        rt.state_bus.get_snapshot(),
    )
    assert allowed is False
    assert "safe mode blocks" in reason


def test_audit_log_records(tmp_path: Path) -> None:
    _, _, audit_path = _run_short_runtime(tmp_path)
    before = len(audit_path.read_text(encoding="utf-8").splitlines())
    # nuevo evento para incrementar
    rt, _, _ = _run_short_runtime(tmp_path, seconds=0.2)
    after = len((tmp_path / "brain_audit_log.jsonl").read_text(encoding="utf-8").splitlines())
    assert after >= before
    assert rt is not None


def test_mode_transitions_smooth(tmp_path: Path) -> None:
    rt, _, _ = _run_short_runtime(tmp_path, seconds=0.2)
    for mode in ("manual", "safe", "recovery", "manual"):
        rt.brain.set_mode(mode)
    snap = rt.state_bus.get_snapshot()
    assert snap.global_mode == "manual"


def test_causal_chain_coherent(tmp_path: Path) -> None:
    _, payload, audit_path = _run_short_runtime(tmp_path)
    events = [json.loads(x) for x in audit_path.read_text(encoding="utf-8").splitlines() if x.strip()]
    types = {e.get("event_type") for e in events}
    assert "event_ingested" in types
    assert "workspace_decision_emitted" in types
    assert ("command_result" in types) or ("command_blocked" in types)
    assert payload["vitals_summary"]["is_alive"] is True
