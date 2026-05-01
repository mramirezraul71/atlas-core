"""Tests F17 — Autonomy FSM (paper-only).

Cubre:

* Camino feliz paper: BOOTING → SCANNING → OPPORTUNITY_DETECTED →
  STRATEGY_BUILDING → BACKTESTING → PAPER_READY → PAPER_EXECUTING →
  MONITORING → EXITING → SCANNING.
* Camino degradado: BOOTING → DEGRADED → SCANNING al recuperar.
* KILL_SWITCH dominante: cualquier estado → KILL_SWITCH →
  ERROR_RECOVERY → BOOTING.
* LIVE_ARMED y LIVE_EXECUTING NO son alcanzables en F17.
* La FSM nunca viola ``ALLOWED_TRANSITIONS``.
* ``LiveArmRequested`` se rechaza sin transicionar.
* ``step()`` es defensivo: nunca propaga excepciones aunque un gate
  rompa internamente.
* AST guard: módulo orchestrator NO importa execution canónico ni
  live_*, ni atlas_adapter, ni operations.live_authorization, etc.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from atlas_code_quant.autonomy.events import (
    DegradationDetected,
    KillSwitchTriggered,
    LiveArmRequested,
    OpportunityArrived,
    PaperExecuted,
    RecoveryDetected,
    StrategyBuilt,
    Tick,
)
from atlas_code_quant.autonomy.gates import (
    BrokerGate,
    Gate,
    GateResult,
    HealthGate,
    KillSwitchGate,
    LiveGate,
    RadarGate,
    RiskGate,
    StrategyGate,
    VisionGate,
)
from atlas_code_quant.autonomy.orchestrator import (
    AutonomyConfig,
    AutonomyOrchestrator,
)
from atlas_code_quant.autonomy.states import (
    ALLOWED_TRANSITIONS,
    LIVE_FORBIDDEN_STATES,
    AutonomyState,
    IllegalTransition,
    assert_transition,
)


# ---------------------------------------------------------------------------
# Sección 1 — Estados y transiciones canónicas
# ---------------------------------------------------------------------------


class TestStatesAndTransitions:
    def test_fourteen_states_present(self):
        assert len(list(AutonomyState)) == 14
        names = {s.name for s in AutonomyState}
        for required in (
            "BOOTING",
            "DEGRADED",
            "SCANNING",
            "OPPORTUNITY_DETECTED",
            "STRATEGY_BUILDING",
            "BACKTESTING",
            "PAPER_READY",
            "PAPER_EXECUTING",
            "MONITORING",
            "EXITING",
            "LIVE_ARMED",
            "LIVE_EXECUTING",
            "KILL_SWITCH",
            "ERROR_RECOVERY",
        ):
            assert required in names

    def test_live_forbidden_in_default_mode(self):
        # default allow_live=False (F17–F20 policy)
        with pytest.raises(IllegalTransition):
            assert_transition(
                AutonomyState.PAPER_READY, AutonomyState.LIVE_ARMED
            )
        with pytest.raises(IllegalTransition):
            assert_transition(
                AutonomyState.LIVE_ARMED, AutonomyState.LIVE_EXECUTING
            )

    def test_legal_paper_transitions(self):
        legal_pairs = [
            (AutonomyState.BOOTING, AutonomyState.SCANNING),
            (AutonomyState.SCANNING, AutonomyState.OPPORTUNITY_DETECTED),
            (AutonomyState.OPPORTUNITY_DETECTED, AutonomyState.STRATEGY_BUILDING),
            (AutonomyState.STRATEGY_BUILDING, AutonomyState.BACKTESTING),
            (AutonomyState.BACKTESTING, AutonomyState.PAPER_READY),
            (AutonomyState.PAPER_READY, AutonomyState.PAPER_EXECUTING),
            (AutonomyState.PAPER_EXECUTING, AutonomyState.MONITORING),
            (AutonomyState.MONITORING, AutonomyState.EXITING),
            (AutonomyState.EXITING, AutonomyState.SCANNING),
            (AutonomyState.KILL_SWITCH, AutonomyState.ERROR_RECOVERY),
            (AutonomyState.ERROR_RECOVERY, AutonomyState.BOOTING),
        ]
        for src, dst in legal_pairs:
            assert_transition(src, dst)

    def test_illegal_jumps_rejected(self):
        with pytest.raises(IllegalTransition):
            assert_transition(
                AutonomyState.BOOTING, AutonomyState.PAPER_EXECUTING
            )
        with pytest.raises(IllegalTransition):
            assert_transition(
                AutonomyState.SCANNING, AutonomyState.MONITORING
            )


# ---------------------------------------------------------------------------
# Sección 2 — Camino feliz paper
# ---------------------------------------------------------------------------


class TestHappyPath:
    def test_full_paper_cycle(self):
        orch = AutonomyOrchestrator()
        # BOOTING → SCANNING
        r = orch.step(Tick())
        assert r.src == AutonomyState.BOOTING
        assert r.dst == AutonomyState.SCANNING

        # SCANNING idle (sin opportunity)
        r = orch.step(Tick())
        assert orch.state == AutonomyState.SCANNING

        # SCANNING → OPPORTUNITY_DETECTED
        r = orch.step(OpportunityArrived(payload={"symbol": "SPY"}))
        assert orch.state == AutonomyState.OPPORTUNITY_DETECTED

        # OPPORTUNITY_DETECTED → STRATEGY_BUILDING
        r = orch.step(Tick())
        assert orch.state == AutonomyState.STRATEGY_BUILDING

        # STRATEGY_BUILDING → BACKTESTING (con StrategyBuilt)
        r = orch.step(StrategyBuilt())
        assert orch.state == AutonomyState.BACKTESTING

        # BACKTESTING → PAPER_READY (con ctx fitness_ready)
        r = orch.step(Tick(), ctx={"fitness_ready": True})
        assert orch.state == AutonomyState.PAPER_READY

        # PAPER_READY → PAPER_EXECUTING (gates stub OK)
        r = orch.step(Tick())
        assert orch.state == AutonomyState.PAPER_EXECUTING

        # PAPER_EXECUTING → MONITORING (PaperExecuted)
        r = orch.step(PaperExecuted())
        assert orch.state == AutonomyState.MONITORING

        # MONITORING → EXITING
        r = orch.step(Tick(), ctx={"monitor_done": True})
        assert orch.state == AutonomyState.EXITING

        # EXITING → SCANNING
        r = orch.step(Tick())
        assert orch.state == AutonomyState.SCANNING

        # Toda la historia debe respetar ALLOWED_TRANSITIONS
        hist = orch.history
        for src, dst in zip(hist, hist[1:]):
            assert dst in ALLOWED_TRANSITIONS.get(
                src, frozenset()
            ), f"transición ilegal {src} → {dst}"


# ---------------------------------------------------------------------------
# Sección 3 — Camino degradado
# ---------------------------------------------------------------------------


class TestDegradedPath:
    def test_boot_with_radar_degraded_goes_to_degraded(self):
        orch = AutonomyOrchestrator()
        r = orch.step(Tick(), ctx={"radar_degraded": True})
        assert orch.state == AutonomyState.DEGRADED

    def test_recover_from_degraded(self):
        orch = AutonomyOrchestrator(initial_state=AutonomyState.DEGRADED)
        # primer tick degradado: sigue degradado
        orch.step(Tick(), ctx={"radar_degraded": True})
        assert orch.state == AutonomyState.DEGRADED
        # tick sin degradación: vuelve a SCANNING
        orch.step(Tick())
        assert orch.state == AutonomyState.SCANNING

    def test_recovery_event_from_degraded(self):
        orch = AutonomyOrchestrator(initial_state=AutonomyState.DEGRADED)
        orch.step(RecoveryDetected())
        assert orch.state == AutonomyState.SCANNING


# ---------------------------------------------------------------------------
# Sección 4 — Kill switch
# ---------------------------------------------------------------------------


class TestKillSwitch:
    def test_killswitch_event_from_any_state(self):
        for start in (
            AutonomyState.SCANNING,
            AutonomyState.OPPORTUNITY_DETECTED,
            AutonomyState.PAPER_READY,
            AutonomyState.MONITORING,
        ):
            orch = AutonomyOrchestrator(initial_state=start)
            orch.step(KillSwitchTriggered())
            assert orch.state == AutonomyState.KILL_SWITCH

    def test_killswitch_via_gate_ctx(self):
        orch = AutonomyOrchestrator(initial_state=AutonomyState.SCANNING)
        orch.step(Tick(), ctx={"killswitch": True})
        assert orch.state == AutonomyState.KILL_SWITCH

    def test_killswitch_to_recovery_to_booting(self):
        orch = AutonomyOrchestrator(initial_state=AutonomyState.KILL_SWITCH)
        # KILL_SWITCH → ERROR_RECOVERY
        orch.step(Tick())
        assert orch.state == AutonomyState.ERROR_RECOVERY
        # ERROR_RECOVERY → BOOTING
        orch.step(Tick())
        assert orch.state == AutonomyState.BOOTING


# ---------------------------------------------------------------------------
# Sección 5 — Live bloqueado en F17
# ---------------------------------------------------------------------------


class TestLiveBlocked:
    def test_live_states_not_reached_in_happy_path(self):
        orch = AutonomyOrchestrator()
        # Ejecutamos un ciclo completo y un par más
        events = [
            Tick(),
            OpportunityArrived(),
            Tick(),
            StrategyBuilt(),
            Tick(),  # backtesting awaits
            Tick(),  # idem
        ]
        for e in events:
            orch.step(e, ctx={"fitness_ready": True})
        # En ningún momento la historia debe contener live
        assert all(s not in LIVE_FORBIDDEN_STATES for s in orch.history)

    def test_live_arm_request_rejected(self):
        orch = AutonomyOrchestrator(initial_state=AutonomyState.PAPER_READY)
        before = orch.state
        r = orch.step(LiveArmRequested())
        assert orch.state == before
        assert "rejected" in r.reason

    def test_live_gate_always_blocks(self):
        gate = LiveGate()
        res = gate.evaluate({})
        assert res.ok is False
        assert "live_blocked" in res.reason


# ---------------------------------------------------------------------------
# Sección 6 — Defensividad
# ---------------------------------------------------------------------------


class _ExplodingGate(Gate):
    name = "explode"

    def evaluate(self, ctx=None):
        raise RuntimeError("simulated gate explosion")


class TestDefensive:
    def test_step_never_raises_on_gate_explosion(self):
        # Sustituimos killswitch por uno que explota
        gates = {
            "health": HealthGate(),
            "radar": RadarGate(),
            "strategy": StrategyGate(),
            "risk": RiskGate(),
            "vision": VisionGate(),
            "broker": BrokerGate(),
            "live": LiveGate(),
            "killswitch": _ExplodingGate(),
        }
        orch = AutonomyOrchestrator(gates=gates)
        # No debe lanzar; el __call__ de Gate ya envuelve excepciones
        r = orch.step(Tick())
        assert r is not None

    def test_step_never_propagates(self):
        orch = AutonomyOrchestrator()
        for _ in range(50):
            orch.step(Tick(), ctx={})
        assert orch.state in set(AutonomyState)


# ---------------------------------------------------------------------------
# Sección 7 — Aislamiento AST
# ---------------------------------------------------------------------------


_F17_MODULE = Path("atlas_code_quant/autonomy/orchestrator.py")


_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution.tradier_execution",
    "atlas_code_quant.execution.broker_router",
    "atlas_code_quant.execution.tradier_controls",
    "atlas_code_quant.execution.tradier_pdt_ledger",
    "atlas_code_quant.operations.auton_executor",
    "atlas_code_quant.operations.live_authorization",
    "atlas_code_quant.operations.live_loop",
    "atlas_code_quant.operations.live_switch",
    "atlas_code_quant.operations.operation_center",
    "atlas_code_quant.operations.signal_executor",
    "atlas_code_quant.operations.start_paper_trading",
    "atlas_code_quant.production.live_activation",
    "atlas_adapter",
)


def _read(path: Path) -> str:
    repo_root = Path(__file__).resolve().parents[2]
    return (repo_root / path).read_text("utf-8")


def test_orchestrator_has_no_prohibited_imports():
    src = _read(_F17_MODULE)
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
    assert not bad, f"orchestrator.py importa prohibidos: {bad}"


def test_orchestrator_does_not_reference_locks_as_code():
    for path in (
        _F17_MODULE,
        Path("atlas_code_quant/autonomy/states.py"),
        Path("atlas_code_quant/autonomy/events.py"),
        Path("atlas_code_quant/autonomy/gates/orchestrator_gates.py"),
    ):
        src = _read(path)
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                assert node.id not in {
                    "paper_only",
                    "full_live_globally_locked",
                }, f"{path} usa lock como Name: {node.id}"
            if isinstance(node, ast.Attribute):
                assert node.attr not in {
                    "paper_only",
                    "full_live_globally_locked",
                }, f"{path} usa lock como Attribute: {node.attr}"
