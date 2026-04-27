"""Tests F8 — 10 gates de seguridad + GateBundle."""
from __future__ import annotations

from atlas_code_quant.autonomy.gates import (
    GateBundle,
    GateContext,
    evaluate_live_gate,
    gate_broker_readiness,
    gate_data_freshness,
    gate_kill_switch_file,
    gate_lean_validation,
    gate_max_daily_loss,
    gate_max_position_notional,
    gate_paper_or_live_permission,
    gate_radar_score,
    gate_risk_engine,
    gate_vision_timing,
)


def _ctx_pass() -> GateContext:
    return GateContext(
        data_age_seconds=10.0,
        radar_score=72.0,
        lean_validated=True,
        risk_decision_allow=True,
        risk_reason="ok",
        vision_decision="allow",
        broker_ready=True,
        paper_mode=True,
        live_enabled=False,
        realized_pnl_today_usd=0.0,
        position_notional_usd=1_000.0,
        kill_switch_file_present=False,
    )


# ── 10 gates individuales ───────────────────────────────────────────────────
def test_gate_data_freshness() -> None:
    ctx = _ctx_pass()
    assert gate_data_freshness(ctx, max_age_seconds=60.0).allow is True
    ctx.data_age_seconds = 120.0
    assert gate_data_freshness(ctx, max_age_seconds=60.0).allow is False


def test_gate_radar_score() -> None:
    ctx = _ctx_pass()
    assert gate_radar_score(ctx, min_score=50.0).allow is True
    ctx.radar_score = 30.0
    assert gate_radar_score(ctx, min_score=50.0).allow is False


def test_gate_lean_validation() -> None:
    ctx = _ctx_pass()
    assert gate_lean_validation(ctx).allow is True
    ctx.lean_validated = False
    assert gate_lean_validation(ctx).allow is False


def test_gate_risk_engine() -> None:
    ctx = _ctx_pass()
    assert gate_risk_engine(ctx).allow is True
    ctx.risk_decision_allow = False
    ctx.risk_reason = "max_open_positions_reached"
    d = gate_risk_engine(ctx)
    assert d.allow is False
    assert "max_open_positions" in d.reason


def test_gate_vision_timing() -> None:
    ctx = _ctx_pass()
    assert gate_vision_timing(ctx).allow is True
    ctx.vision_decision = "delay"
    assert gate_vision_timing(ctx).allow is False
    ctx.vision_decision = "block"
    assert gate_vision_timing(ctx).allow is False


def test_gate_broker_readiness() -> None:
    ctx = _ctx_pass()
    assert gate_broker_readiness(ctx).allow is True
    ctx.broker_ready = False
    assert gate_broker_readiness(ctx).allow is False


def test_gate_paper_or_live_permission() -> None:
    ctx = _ctx_pass()
    assert gate_paper_or_live_permission(ctx).allow is True
    ctx.paper_mode = False
    ctx.live_enabled = False
    d = gate_paper_or_live_permission(ctx)
    assert d.allow is False
    assert d.reason == "live_disabled_by_flag"
    ctx.live_enabled = True
    assert gate_paper_or_live_permission(ctx).allow is True


def test_gate_max_daily_loss() -> None:
    ctx = _ctx_pass()
    assert gate_max_daily_loss(ctx, max_loss_usd=1_000.0).allow is True
    ctx.realized_pnl_today_usd = -1_001.0
    assert gate_max_daily_loss(ctx, max_loss_usd=1_000.0).allow is False


def test_gate_max_position_notional() -> None:
    ctx = _ctx_pass()
    assert gate_max_position_notional(ctx, max_notional_usd=5_000.0).allow is True
    ctx.position_notional_usd = 6_000.0
    assert gate_max_position_notional(ctx, max_notional_usd=5_000.0).allow is False


def test_gate_kill_switch_file() -> None:
    ctx = _ctx_pass()
    assert gate_kill_switch_file(ctx).allow is True
    ctx.kill_switch_file_present = True
    assert gate_kill_switch_file(ctx).allow is False


# ── Bundle ──────────────────────────────────────────────────────────────────
def test_bundle_passes_when_all_ok() -> None:
    bundle = GateBundle()
    d = bundle.evaluate(_ctx_pass())
    assert d.allow is True
    assert d.reason == "all_gates_pass"


def test_bundle_short_circuits_on_kill_switch_first() -> None:
    bundle = GateBundle()
    ctx = _ctx_pass()
    ctx.kill_switch_file_present = True
    ctx.broker_ready = False  # también fallaría, pero kill switch primero
    d = bundle.evaluate(ctx)
    assert d.allow is False
    assert d.reason == "kill_switch_file_present"


def test_bundle_returns_first_blocker() -> None:
    bundle = GateBundle()
    ctx = _ctx_pass()
    ctx.broker_ready = False
    d = bundle.evaluate(ctx)
    assert d.allow is False
    assert d.reason == "broker_not_ready"


def test_bundle_default_has_10_gates() -> None:
    bundle = GateBundle()
    assert len(bundle.gates) == 10


# ── Compat F1 ───────────────────────────────────────────────────────────────
def test_evaluate_live_gate_compat() -> None:
    assert evaluate_live_gate(False).allow is False
    assert evaluate_live_gate(True).allow is True
