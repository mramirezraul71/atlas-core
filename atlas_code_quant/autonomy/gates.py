"""10 Gates de seguridad — F8.

Cada gate devuelve un ``GateDecision(allow, reason)``. El ``GateBundle``
encadena los gates y devuelve la primera razón de bloqueo para auditoría.

Compatibilidad F1: ``GateDecision`` y ``evaluate_live_gate`` se conservan.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable


def _env_bool(name: str, default: bool) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


@dataclass(slots=True)
class GateDecision:
    allow: bool
    reason: str = "f1_stub"


# ── F1 compat ───────────────────────────────────────────────────────────────
def evaluate_live_gate(live_enabled: bool) -> GateDecision:
    if not live_enabled:
        return GateDecision(allow=False, reason="ATLAS_LIVE_TRADING_ENABLED=false")
    return GateDecision(allow=True, reason="live_explicitly_enabled")


# ── F8 contexto ─────────────────────────────────────────────────────────────
@dataclass(slots=True)
class GateContext:
    # Datos
    data_age_seconds: float = 0.0
    radar_score: float = 0.0
    # Validaciones
    lean_validated: bool = False
    risk_decision_allow: bool = True
    risk_reason: str = "ok"
    vision_decision: str = "allow"           # allow|delay|block|force_exit
    broker_ready: bool = True
    # Permisos
    paper_mode: bool = True                  # True = paper, False = live
    live_enabled: bool = False
    # Métricas riesgo
    realized_pnl_today_usd: float = 0.0
    position_notional_usd: float = 0.0
    # Kill switch
    kill_switch_file_present: bool = False


# ── 10 Gates ────────────────────────────────────────────────────────────────
def gate_data_freshness(ctx: GateContext, max_age_seconds: float = 60.0) -> GateDecision:
    if ctx.data_age_seconds > max_age_seconds:
        return GateDecision(False, f"data_stale>{max_age_seconds}s")
    return GateDecision(True, "fresh")


def gate_radar_score(ctx: GateContext, min_score: float = 50.0) -> GateDecision:
    if ctx.radar_score < min_score:
        return GateDecision(False, f"radar_score<{min_score}")
    return GateDecision(True, "radar_ok")


def gate_lean_validation(ctx: GateContext) -> GateDecision:
    if not ctx.lean_validated:
        return GateDecision(False, "lean_not_validated")
    return GateDecision(True, "lean_validated")


def gate_risk_engine(ctx: GateContext) -> GateDecision:
    if not ctx.risk_decision_allow:
        return GateDecision(False, f"risk:{ctx.risk_reason}")
    return GateDecision(True, "risk_ok")


def gate_vision_timing(ctx: GateContext) -> GateDecision:
    if ctx.vision_decision in {"block", "force_exit"}:
        return GateDecision(False, f"vision:{ctx.vision_decision}")
    if ctx.vision_decision == "delay":
        return GateDecision(False, "vision_delay")
    return GateDecision(True, "vision_allow")


def gate_broker_readiness(ctx: GateContext) -> GateDecision:
    if not ctx.broker_ready:
        return GateDecision(False, "broker_not_ready")
    return GateDecision(True, "broker_ready")


def gate_paper_or_live_permission(ctx: GateContext) -> GateDecision:
    if ctx.paper_mode:
        return GateDecision(True, "paper_mode")
    if not ctx.live_enabled:
        return GateDecision(False, "live_disabled_by_flag")
    return GateDecision(True, "live_enabled")


def gate_max_daily_loss(ctx: GateContext, max_loss_usd: float = 1_000.0) -> GateDecision:
    if ctx.realized_pnl_today_usd <= -abs(max_loss_usd):
        return GateDecision(False, "max_daily_loss_breached")
    return GateDecision(True, "daily_loss_ok")


def gate_max_position_notional(ctx: GateContext, max_notional_usd: float = 5_000.0) -> GateDecision:
    if ctx.position_notional_usd > max_notional_usd:
        return GateDecision(False, "position_notional_exceeds_limit")
    return GateDecision(True, "notional_ok")


def gate_kill_switch_file(ctx: GateContext) -> GateDecision:
    if ctx.kill_switch_file_present:
        return GateDecision(False, "kill_switch_file_present")
    return GateDecision(True, "kill_switch_off")


# ── Bundle ──────────────────────────────────────────────────────────────────
GateFn = Callable[[GateContext], GateDecision]


@dataclass(slots=True)
class GateBundle:
    """Evalúa los 10 gates en orden y devuelve la primera razón de bloqueo."""

    gates: list[GateFn] = field(
        default_factory=lambda: [
            gate_kill_switch_file,
            gate_broker_readiness,
            gate_data_freshness,
            gate_radar_score,
            gate_lean_validation,
            gate_risk_engine,
            gate_max_daily_loss,
            gate_max_position_notional,
            gate_vision_timing,
            gate_paper_or_live_permission,
        ]
    )

    def evaluate(self, ctx: GateContext) -> GateDecision:
        for fn in self.gates:
            decision = fn(ctx)
            if not decision.allow:
                return decision
        return GateDecision(True, "all_gates_pass")
