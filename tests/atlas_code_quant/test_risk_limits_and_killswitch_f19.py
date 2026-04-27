"""Tests F19 — Risk limits, circuit breaker y kill switch (paper-only).

Cubre:

* `check_daily_loss_limit` / `check_position_notional_limit` /
  `check_orders_per_minute` con valores dentro y fuera de límites.
* `check_all_limits` falla rápido al primer violador.
* `load_risk_limits_from_env` parsea env vars con defaults seguros.
* `CircuitBreaker`: closed → open al alcanzar threshold, cooldown
  → half_open, success → closed.
* `CircuitBreaker.trip` / `reset` manuales.
* `FileKillSwitch`: file ausente → no activado; file con contenido
  → activado; file vacío → no activado; file no legible defensivo.
* `RiskGate` integrado: violación + breaker registra failure;
  breaker abierto bloquea sin re-checar.
* `KillSwitchGate` integrado: override ctx prioritario; estado real
  por fichero.
* FSM reacciona: cualquier estado + killswitch=True → KILL_SWITCH.
* AST guards: módulos F19 no importan execution / live / atlas_adapter.
"""

from __future__ import annotations

import ast
import os
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
from atlas_code_quant.autonomy.states import AutonomyState
from atlas_code_quant.autonomy.events import KillSwitchTriggered, Tick
from atlas_code_quant.risk.circuit_breaker import (
    BreakerState,
    CircuitBreaker,
    CircuitBreakerConfig,
)
from atlas_code_quant.risk.kill_switch import (
    FileKillSwitch,
    KillSwitchStatus,
    load_kill_switch_path_from_env,
)
from atlas_code_quant.risk.limits import (
    RiskLimitsConfig,
    check_all_limits,
    check_daily_loss_limit,
    check_orders_per_minute,
    check_position_notional_limit,
    load_risk_limits_from_env,
)


# ---------------------------------------------------------------------------
# Sección 1 — Risk limits
# ---------------------------------------------------------------------------


class TestRiskLimits:
    def test_daily_loss_within(self):
        cfg = RiskLimitsConfig(max_daily_loss_usd=500)
        r = check_daily_loss_limit(realized_pnl_usd=-499.99, config=cfg)
        assert r.ok is True

    def test_daily_loss_exceeded(self):
        cfg = RiskLimitsConfig(max_daily_loss_usd=500)
        r = check_daily_loss_limit(realized_pnl_usd=-501, config=cfg)
        assert r.ok is False
        assert "exceeded" in r.reason

    def test_notional_within(self):
        cfg = RiskLimitsConfig(max_position_notional_usd=2_500)
        r = check_position_notional_limit(notional_usd=2_500, config=cfg)
        assert r.ok is True

    def test_notional_exceeded(self):
        cfg = RiskLimitsConfig(max_position_notional_usd=2_500)
        r = check_position_notional_limit(notional_usd=2_500.01, config=cfg)
        assert r.ok is False

    def test_orders_per_minute_within(self):
        cfg = RiskLimitsConfig(max_orders_per_minute=10)
        r = check_orders_per_minute(orders_in_last_minute=9, config=cfg)
        assert r.ok is True

    def test_orders_per_minute_exceeded(self):
        cfg = RiskLimitsConfig(max_orders_per_minute=10)
        r = check_orders_per_minute(orders_in_last_minute=10, config=cfg)
        assert r.ok is False

    def test_check_all_short_circuits_first_violation(self):
        cfg = RiskLimitsConfig(
            max_daily_loss_usd=100,
            max_position_notional_usd=100,
            max_orders_per_minute=1,
        )
        r = check_all_limits(
            realized_pnl_usd=-200,  # primer violador
            notional_usd=999,
            orders_in_last_minute=999,
            config=cfg,
        )
        assert r.ok is False
        assert "daily_loss" in r.reason

    def test_check_all_passes(self):
        cfg = RiskLimitsConfig(
            max_daily_loss_usd=100,
            max_position_notional_usd=100,
            max_orders_per_minute=1,
        )
        r = check_all_limits(
            realized_pnl_usd=-50,
            notional_usd=50,
            orders_in_last_minute=0,
            config=cfg,
        )
        assert r.ok is True

    def test_load_from_env_defaults(self, monkeypatch):
        for k in (
            "ATLAS_MAX_DAILY_LOSS_USD",
            "ATLAS_MAX_POSITION_NOTIONAL_USD",
            "ATLAS_MAX_ORDERS_PER_MINUTE",
        ):
            monkeypatch.delenv(k, raising=False)
        cfg = load_risk_limits_from_env()
        assert cfg.max_daily_loss_usd == 500.0
        assert cfg.max_position_notional_usd == 2_500.0
        assert cfg.max_orders_per_minute == 30

    def test_load_from_env_custom(self, monkeypatch):
        monkeypatch.setenv("ATLAS_MAX_DAILY_LOSS_USD", "123.5")
        monkeypatch.setenv("ATLAS_MAX_POSITION_NOTIONAL_USD", "777")
        monkeypatch.setenv("ATLAS_MAX_ORDERS_PER_MINUTE", "5")
        cfg = load_risk_limits_from_env()
        assert cfg.max_daily_loss_usd == 123.5
        assert cfg.max_position_notional_usd == 777.0
        assert cfg.max_orders_per_minute == 5


# ---------------------------------------------------------------------------
# Sección 2 — Circuit breaker
# ---------------------------------------------------------------------------


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class TestCircuitBreaker:
    def test_starts_closed(self):
        cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=2))
        assert cb.state == BreakerState.CLOSED
        assert cb.allow_request() is True

    def test_open_after_threshold(self):
        cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=3))
        for _ in range(2):
            cb.record_failure()
        assert cb.state == BreakerState.CLOSED
        cb.record_failure()
        assert cb.state == BreakerState.OPEN
        assert cb.allow_request() is False

    def test_success_resets_counter(self):
        cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=3))
        cb.record_failure()
        cb.record_failure()
        cb.record_success()
        assert cb.consecutive_failures == 0
        assert cb.state == BreakerState.CLOSED

    def test_cooldown_to_half_open(self):
        clock = _Clock()
        cb = CircuitBreaker(
            config=CircuitBreakerConfig(
                failure_threshold=1, cooldown_seconds=10
            ),
            time_fn=clock,
        )
        cb.record_failure()
        assert cb.state == BreakerState.OPEN
        # Antes del cooldown sigue OPEN
        clock.t = 5
        assert cb.state == BreakerState.OPEN
        # Tras cooldown pasa a HALF_OPEN
        clock.t = 11
        assert cb.state == BreakerState.HALF_OPEN
        assert cb.allow_request() is True

    def test_half_open_failure_reopens(self):
        clock = _Clock()
        cb = CircuitBreaker(
            config=CircuitBreakerConfig(
                failure_threshold=1, cooldown_seconds=10
            ),
            time_fn=clock,
        )
        cb.record_failure()
        clock.t = 11
        assert cb.state == BreakerState.HALF_OPEN
        cb.record_failure()
        assert cb.state == BreakerState.OPEN

    def test_manual_trip_and_reset(self):
        cb = CircuitBreaker()
        cb.trip("manual")
        assert cb.state == BreakerState.OPEN
        cb.reset()
        assert cb.state == BreakerState.CLOSED
        assert cb.consecutive_failures == 0


# ---------------------------------------------------------------------------
# Sección 3 — File kill switch
# ---------------------------------------------------------------------------


class TestFileKillSwitch:
    def test_file_absent(self, tmp_path):
        sw = FileKillSwitch(path=str(tmp_path / "ks"))
        st = sw.status()
        assert st.activated is False
        assert "absent" in st.reason

    def test_file_with_kill(self, tmp_path):
        p = tmp_path / "ks"
        p.write_text("KILL")
        sw = FileKillSwitch(path=str(p))
        st = sw.status()
        assert st.activated is True
        assert st.raw_marker == "KILL"
        assert sw.is_activated() is True

    def test_file_empty_not_activated(self, tmp_path):
        p = tmp_path / "ks"
        p.write_text("")
        sw = FileKillSwitch(path=str(p))
        st = sw.status()
        assert st.activated is False
        assert "empty" in st.reason

    def test_create_and_delete_round_trip(self, tmp_path):
        p = tmp_path / "ks"
        sw = FileKillSwitch(path=str(p))
        assert sw.is_activated() is False
        p.write_text("KILL")
        assert sw.is_activated() is True
        p.unlink()
        assert sw.is_activated() is False

    def test_path_provider(self, tmp_path):
        p = tmp_path / "ks"
        p.write_text("STOP")
        sw = FileKillSwitch(path_provider=lambda: str(p))
        assert sw.is_activated() is True

    def test_load_path_default(self, monkeypatch):
        monkeypatch.delenv("ATLAS_KILLSWITCH_FILE", raising=False)
        assert load_kill_switch_path_from_env(default="/tmp/x") == "/tmp/x"
        monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", "/tmp/foo")
        assert load_kill_switch_path_from_env() == "/tmp/foo"


# ---------------------------------------------------------------------------
# Sección 4 — RiskGate integrado
# ---------------------------------------------------------------------------


class TestRiskGateIntegrated:
    def test_no_breaker_violation_returns_block(self):
        gate = RiskGate(
            config=RiskLimitsConfig(max_daily_loss_usd=10),
        )
        r = gate.evaluate({"realized_pnl_usd": -100})
        assert r.ok is False

    def test_breaker_records_failures_and_opens(self):
        cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=2))
        gate = RiskGate(
            breaker=cb,
            config=RiskLimitsConfig(max_daily_loss_usd=10),
        )
        gate.evaluate({"realized_pnl_usd": -100})
        gate.evaluate({"realized_pnl_usd": -100})
        assert cb.state == BreakerState.OPEN
        # con breaker abierto, el gate bloquea sin re-checar
        r = gate.evaluate({"realized_pnl_usd": 0})
        assert r.ok is False
        assert "breaker" in r.reason

    def test_breaker_recovers_with_success(self):
        cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=3))
        gate = RiskGate(
            breaker=cb,
            config=RiskLimitsConfig(max_daily_loss_usd=10),
        )
        gate.evaluate({"realized_pnl_usd": -100})  # fail
        gate.evaluate({"realized_pnl_usd": 0})  # success → reset counter
        assert cb.consecutive_failures == 0


# ---------------------------------------------------------------------------
# Sección 5 — KillSwitchGate integrado en la FSM
# ---------------------------------------------------------------------------


class TestKillSwitchGateInFSM:
    def test_ctx_override_takes_priority(self):
        gate = KillSwitchGate(switch=None)
        r = gate.evaluate({"killswitch": True})
        assert r.ok is False

    def test_real_file_triggers(self, tmp_path):
        p = tmp_path / "ks"
        p.write_text("KILL")
        gate = KillSwitchGate(switch=FileKillSwitch(path=str(p)))
        r = gate.evaluate({})
        assert r.ok is False
        assert "killswitch_file_present" in r.reason

    def test_real_file_absent_clears(self, tmp_path):
        gate = KillSwitchGate(switch=FileKillSwitch(path=str(tmp_path / "ks")))
        r = gate.evaluate({})
        assert r.ok is True

    def test_fsm_reacts_to_real_file(self, tmp_path):
        p = tmp_path / "ks"
        gates = {
            "health": HealthGate(),
            "radar": RadarGate(),
            "strategy": StrategyGate(),
            "risk": RiskGate(),
            "vision": VisionGate(),
            "broker": BrokerGate(),
            "live": LiveGate(),
            "killswitch": KillSwitchGate(switch=FileKillSwitch(path=str(p))),
        }
        orch = AutonomyOrchestrator(
            gates=gates, initial_state=AutonomyState.SCANNING
        )
        # Sin fichero → sigue avanzando.
        orch.step(Tick())
        assert orch.state != AutonomyState.KILL_SWITCH
        # Con fichero → fuerza kill switch.
        p.write_text("KILL")
        orch.step(Tick())
        assert orch.state == AutonomyState.KILL_SWITCH


# ---------------------------------------------------------------------------
# Sección 6 — AST guards
# ---------------------------------------------------------------------------


_F19_MODULES = (
    Path("atlas_code_quant/risk/limits/checks.py"),
    Path("atlas_code_quant/risk/kill_switch/file_switch.py"),
    Path("atlas_code_quant/risk/circuit_breaker.py"),
)


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


def test_f19_modules_isolation():
    repo_root = Path(__file__).resolve().parents[2]
    for path in _F19_MODULES:
        src = (repo_root / path).read_text("utf-8")
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
        assert not bad, f"{path} importa prohibidos: {bad}"


def test_f19_modules_dont_reference_locks():
    repo_root = Path(__file__).resolve().parents[2]
    for path in _F19_MODULES:
        src = (repo_root / path).read_text("utf-8")
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
