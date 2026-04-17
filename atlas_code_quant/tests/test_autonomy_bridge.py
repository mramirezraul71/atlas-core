from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.autonomy_bridge import AutonomyBridge
from atlas_core.autonomy.models import ModuleRisks as LegacyRisks
from atlas_core.autonomy.models import ModuleState as LegacyState
from atlas_core.autonomy.models import Snapshot as LegacySnapshot


class _FakeLegacyStateBus:
    def collect_snapshot(self):
        return LegacySnapshot(
            modules={
                "quant": {
                    "state": LegacyState(name="quant", mode="semi", health="ok", details={"x": 1}),
                    "risks": LegacyRisks(name="quant", risks={"drawdown": "high"}),
                    "capabilities": {},
                }
            }
        )


def test_autonomy_bridge_default_read_only() -> None:
    br = AutonomyBridge(state_bus=_FakeLegacyStateBus())
    status = br.status()
    assert status.canonical_controller == "brain"
    assert status.legacy_control_enabled is False
    out = br.apply_legacy_command({"target": "quant", "action": "set_mode", "params": {"mode": "safe"}})
    assert out["ok"] is False


def test_autonomy_bridge_maps_legacy_snapshot() -> None:
    br = AutonomyBridge(state_bus=_FakeLegacyStateBus())
    states, risks = br.as_brain_states()
    assert "quant" in states
    assert states["quant"].mode == "semi"
    assert "quant" in risks

