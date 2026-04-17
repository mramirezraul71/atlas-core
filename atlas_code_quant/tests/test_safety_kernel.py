from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import Command, ModuleState, RiskState, SystemSnapshot
from atlas_core.brain.safety_kernel import SafetyKernel
from atlas_core.brain.state_bus import StateBus


def test_manual_blocks_active_body_command() -> None:
    sk = SafetyKernel()
    snap = SystemSnapshot(
        global_mode="manual",
        active_mission=None,
        modules={},
        risks={},
        meta={},
    )
    ok, _ = sk.authorize(Command(target="body", action="punch", params={}), snap)
    assert ok is False


def test_quant_critical_blocks_aggressive() -> None:
    sk = SafetyKernel()
    snap = SystemSnapshot(
        global_mode="autonomous",
        active_mission=None,
        modules={},
        risks={"quant": RiskState(name="quant", level="critical", details={})},
        meta={},
    )
    ok, _ = sk.authorize(Command(target="quant", action="submit", params={}), snap)
    assert ok is False


def test_trigger_fail_safe_sets_safe() -> None:
    sk = SafetyKernel()
    bus = StateBus()
    bus.set_global_mode("autonomous")
    sk.trigger_fail_safe("test_reason", bus)
    assert bus.get_snapshot().global_mode == "safe"
