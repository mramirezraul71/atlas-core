from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import ModuleState, RiskState
from atlas_core.brain.state_bus import StateBus


def test_state_bus_snapshot() -> None:
    bus = StateBus()
    bus.set_global_mode("autonomous")
    bus.set_active_mission("trade_scan")
    bus.update_module_state(
        "quant",
        ModuleState(name="quant", health="ok", mode="semi", details={}),
    )
    bus.update_risk("quant", RiskState(name="quant", level="low", details={}))
    bus.record_execution_result("quant", {"ok": True})

    snap = bus.get_snapshot()
    assert snap.global_mode == "autonomous"
    assert snap.active_mission == "trade_scan"
    assert snap.modules["quant"].health == "ok"
    assert snap.risks["quant"].level == "low"
    assert snap.meta.get("execution_count") == 1
