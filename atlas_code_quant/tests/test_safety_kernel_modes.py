from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import Command, ModuleState, RiskState, SystemSnapshot
from atlas_core.brain.safety_kernel import SafetyKernel


def _snap(mode: str, *, body_health: str = "ok", quant_risk: str = "low", sh_risk: str = "low") -> SystemSnapshot:
    return SystemSnapshot(
        global_mode=mode,
        active_mission=None,
        modules={"body": ModuleState(name="body", health=body_health, mode="idle", details={})},
        risks={
            "quant": RiskState(name="quant", level=quant_risk, details={}),
            "system_health": RiskState(name="system_health", level=sh_risk, details={}),
        },
        meta={},
    )


def test_manual_mode_allows_only_manual_actions() -> None:
    sk = SafetyKernel()
    ok1, _ = sk.authorize(Command(target="body", action="speak", params={}), _snap("manual"))
    ok2, _ = sk.authorize(Command(target="quant", action="submit", params={}), _snap("manual"))
    assert ok1 is True
    assert ok2 is False


def test_safe_mode_blocks_quant_submit() -> None:
    sk = SafetyKernel()
    ok, reason = sk.authorize(Command(target="quant", action="submit", params={}), _snap("safe"))
    assert ok is False
    assert "safe mode" in reason


def test_recovery_mode_blocks_quant_aggressive() -> None:
    sk = SafetyKernel()
    ok, reason = sk.authorize(Command(target="quant", action="open_trade", params={}), _snap("recovery"))
    assert ok is False
    assert "recovery mode" in reason


def test_system_health_critical_limits_commands() -> None:
    sk = SafetyKernel()
    ok, reason = sk.authorize(
        Command(target="quant", action="reduce_risk", params={}),
        _snap("autonomous", sh_risk="critical"),
    )
    assert ok is False
    assert "system_health critical" in reason

