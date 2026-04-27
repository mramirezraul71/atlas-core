"""Tests F18 — VisionGate dentro de la FSM (paper-only).

Cubre:

* PAPER_READY + VisionGate ALLOW → PAPER_EXECUTING.
* PAPER_READY + VisionGate DELAY → permanece en PAPER_READY.
* PAPER_READY + VisionGate BLOCK → STRATEGY_BUILDING.
* PAPER_READY + VisionGate FORCE_EXIT → EXITING.
* Cámara unavailable + requires_visual_confirmation=False → ALLOW
  (degraded=True). FSM avanza con flag degraded.
* Cámara unavailable + requires_visual_confirmation=True → BLOCK →
  STRATEGY_BUILDING.
* Live sigue inalcanzable aun pasando todos los gates de visión.
* AST guard: vision/timing_gate/gate.py NO importa execution / live.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from atlas_code_quant.autonomy.gates import (
    BrokerGate,
    HealthGate,
    KillSwitchGate,
    LiveGate,
    RadarGate,
    RiskGate,
    StrategyGate,
    VisionGate,
)
from atlas_code_quant.autonomy.orchestrator import AutonomyOrchestrator
from atlas_code_quant.autonomy.states import LIVE_FORBIDDEN_STATES, AutonomyState
from atlas_code_quant.vision.timing_gate import (
    GateInput,
    GateOutput,
    VisionDecision,
    VisionTimingGate,
)


def _gates_with_timing(timing_gate):
    return {
        "health": HealthGate(),
        "radar": RadarGate(),
        "strategy": StrategyGate(),
        "risk": RiskGate(),
        "vision": VisionGate(timing_gate=timing_gate),
        "broker": BrokerGate(),
        "live": LiveGate(),
        "killswitch": KillSwitchGate(),
    }


def _make_timing(decision: VisionDecision, *, degraded: bool = False, camera_status: str = "ok"):
    def ev(inp, cap):
        return GateOutput(
            decision=decision,
            reason=f"{decision.value}_test",
            degraded=degraded,
            camera_status=camera_status,
        )

    return VisionTimingGate(
        capture_probe=lambda: {"ok": True, "source": "test", "error": None},
        pattern_evaluator=ev,
    )


class TestVisionInFSM:
    def test_allow_advances_to_paper_executing(self):
        gates = _gates_with_timing(_make_timing(VisionDecision.ALLOW))
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        orch.step()
        assert orch.state == AutonomyState.PAPER_EXECUTING

    def test_delay_stays_in_paper_ready(self):
        gates = _gates_with_timing(_make_timing(VisionDecision.DELAY))
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        r = orch.step()
        assert orch.state == AutonomyState.PAPER_READY
        assert "vision_delay" in r.reason

    def test_block_goes_back_to_strategy_building(self):
        gates = _gates_with_timing(_make_timing(VisionDecision.BLOCK))
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        orch.step()
        assert orch.state == AutonomyState.STRATEGY_BUILDING

    def test_force_exit_transitions_to_exiting(self):
        gates = _gates_with_timing(_make_timing(VisionDecision.FORCE_EXIT))
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        orch.step()
        assert orch.state == AutonomyState.EXITING


class TestCameraUnavailable:
    def test_unavailable_optional_allows_degraded(self):
        # ALLOW + degraded=True
        gates = _gates_with_timing(
            _make_timing(
                VisionDecision.ALLOW, degraded=True, camera_status="unavailable"
            )
        )
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        r = orch.step()
        assert orch.state == AutonomyState.PAPER_EXECUTING
        # el flag degraded se debe propagar al tick result
        assert r.degraded is True

    def test_unavailable_required_blocks(self):
        gates = _gates_with_timing(
            _make_timing(
                VisionDecision.BLOCK, degraded=True, camera_status="unavailable"
            )
        )
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.PAPER_READY
        )
        orch.step()
        assert orch.state == AutonomyState.STRATEGY_BUILDING


class TestLiveStillBlocked:
    def test_no_path_reaches_live_even_with_allow(self):
        gates = _gates_with_timing(_make_timing(VisionDecision.ALLOW))
        orch = AutonomyOrchestrator(gates=gates)
        # Ciclo completo
        from atlas_code_quant.autonomy.events import (
            OpportunityArrived,
            PaperExecuted,
            StrategyBuilt,
            Tick,
        )

        for ev in [
            Tick(),
            OpportunityArrived(),
            Tick(),
            StrategyBuilt(),
            Tick(),
            Tick(),  # idem
        ]:
            orch.step(ev, ctx={"fitness_ready": True})
        assert all(s not in LIVE_FORBIDDEN_STATES for s in orch.history)


# ---------------------------------------------------------------------------
# AST guard
# ---------------------------------------------------------------------------


_F18_MODULE = Path("atlas_code_quant/vision/timing_gate/gate.py")


_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution.tradier_execution",
    "atlas_code_quant.execution.broker_router",
    "atlas_code_quant.execution.tradier_controls",
    "atlas_code_quant.execution.tradier_pdt_ledger",
    "atlas_code_quant.operations.live_authorization",
    "atlas_code_quant.operations.live_loop",
    "atlas_code_quant.operations.live_switch",
    "atlas_code_quant.operations.operation_center",
    "atlas_code_quant.operations.signal_executor",
    "atlas_code_quant.production.live_activation",
    "atlas_adapter",
)


def test_timing_gate_module_isolation():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F18_MODULE).read_text("utf-8")
    tree = ast.parse(src)
    bad: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for prohibited in _PROHIBITED_IMPORTS:
                if mod == prohibited or mod.startswith(prohibited + "."):
                    bad.append(mod)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                for prohibited in _PROHIBITED_IMPORTS:
                    if alias.name == prohibited or alias.name.startswith(
                        prohibited + "."
                    ):
                        bad.append(alias.name)
    assert not bad, f"timing_gate/gate.py importa prohibidos: {bad}"


def test_timing_gate_does_not_reference_locks():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F18_MODULE).read_text("utf-8")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            assert node.id not in {
                "paper_only",
                "full_live_globally_locked",
            }
        if isinstance(node, ast.Attribute):
            assert node.attr not in {
                "paper_only",
                "full_live_globally_locked",
            }
