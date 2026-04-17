from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import Event
from atlas_core.runtime.bootstrap import build_brain_system


def test_operator_speak_to_body() -> None:
    rt = build_brain_system()
    rt.brain.set_mode("semi")
    rt.brain.ingest_event(Event(source="operator", kind="speak", payload={"text": "hola"}))
    results = rt.brain.evaluate()
    assert any(r.get("ok") for r in results)


def test_system_critical_health_fail_safe() -> None:
    rt = build_brain_system()
    rt.brain.set_mode("autonomous")
    rt.brain.ingest_event(Event(source="system", kind="critical_health", payload={"reason": "disk"}))
    rt.brain.evaluate()
    assert rt.state_bus.get_snapshot().global_mode == "safe"


def test_operator_set_mission() -> None:
    rt = build_brain_system()
    rt.brain.ingest_event(
        Event(source="operator", kind="set_mission", payload={"mission_name": "alpha", "priority": 1})
    )
    rt.brain.evaluate()
    assert rt.mission_manager.get_active_mission() == "alpha"
    assert rt.state_bus.get_snapshot().active_mission == "alpha"
