from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import Command, Event
from atlas_core.brain.physiological_dashboard import PhysiologicalDashboard
from atlas_core.runtime.bootstrap import build_brain_system
from atlas_core.runtime.event_loop import start_event_loop_background
from atlas_core.runtime.heartbeat_loop import start_heartbeat_background


def test_atlas_physiological_coherence(tmp_path: Path) -> None:
    op_state = tmp_path / "operation_center_state.json"
    op_state.write_text("{}", encoding="utf-8")
    audit_path = tmp_path / "brain_audit_log.jsonl"

    rt = build_brain_system(operation_center_path=op_state)
    rt.audit_log.path = audit_path

    stop = threading.Event()
    hb = start_heartbeat_background(rt.adapters, rt.state_bus, stop, interval_sec=0.05)
    ev = start_event_loop_background(rt.brain, stop, interval_sec=0.05)

    rt.brain.ingest_event(Event(source="operator", kind="speak", payload={"text": "atlas"}))
    rt.brain.ingest_event(Event(source="quant", kind="risk_alert", payload={"mode": "safe"}))
    time.sleep(0.35)
    stop.set()
    hb.join(timeout=2.0)
    ev.join(timeout=2.0)

    dashboard = PhysiologicalDashboard(
        state_bus=rt.state_bus,
        workspace_bridge=rt.workspace_bridge,
        audit_log_path=rt.audit_log.path,
        learning_root=Path(r"c:\ATLAS_PUSH\atlas_code_quant\data\learning"),
    )
    payload = dashboard.snapshot()

    # 1) Heartbeat regular (sin atascos)
    assert payload["heartbeat"]["bpm"] > 0
    assert payload["heartbeat"]["rhythm"] == "regular"

    # 2) Brain Core responde en < 500ms
    assert int(payload["nervous_system"]["latency_brain_ms"]) < 500

    # 3) SafetyKernel bloquea según reglas
    rt.brain.set_mode("safe")
    snap = rt.state_bus.get_snapshot()
    allowed, _ = rt.safety_kernel.authorize(Command(target="quant", action="submit", params={}), snap)
    assert allowed is False

    # 4) Audit log registra eventos importantes
    lines = [json.loads(x) for x in audit_path.read_text(encoding="utf-8").splitlines() if x.strip()]
    event_types = {row.get("event_type") for row in lines}
    assert "event_ingested" in event_types
    assert "command_result" in event_types or "command_blocked" in event_types

    # 5) Transiciones de modo suaves
    rt.brain.set_mode("manual")
    rt.brain.set_mode("safe")
    rt.brain.set_mode("recovery")
    assert rt.state_bus.get_snapshot().global_mode == "recovery"

    # 6) Cadena causal coherente de decisiones
    assert float(payload["brain_state"]["cerebral_activity"]) >= 0.0
    assert int(payload["memory"]["episodes_recorded"]) >= 2
