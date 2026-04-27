"""Tests del :mod:`modules.atlas_radar_kalshi.risk_engine`.

Cubre:
- Fórmula de Kelly (casos de borde y monotonía).
- Caps por posición / mercado / total.
- Breaker por drawdown diario / semanal.
- Breaker por pérdidas consecutivas.
- Breaker por rate-limit (max_orders_per_minute).
- Kill-switch manual y por env ``ATLAS_RADAR_KILL=1``.
- Comportamiento de :meth:`size` cuando ``gate.accepted=False``.
"""
from __future__ import annotations

import os
import time

import pytest

from modules.atlas_radar_kalshi.gating import GateDecision
from modules.atlas_radar_kalshi.risk_engine import (
    RiskEngine,
    RiskLimits,
    RiskState,
)
from modules.atlas_radar_kalshi.signals import SignalReadout


def _gate(accepted: bool = True, side: str = "YES",
          price_cents: int = 50, edge_net: float = 0.05) -> GateDecision:
    return GateDecision(
        accepted=accepted, reason="ok" if accepted else "rejected",
        edge_gross=edge_net, edge_net=edge_net, score=edge_net,
        side=side if accepted else None, price_cents=price_cents,
    )


def _readout(p: float = 0.7) -> SignalReadout:
    return SignalReadout(
        p_micro=p, p_markov=p, p_llm=p, p_momentum=p, p_ensemble=p,
        confidence=0.8, liquidity_score=0.8, spread_ticks=2,
        microprice=p * 100, depth_yes=200, depth_no=200,
    )


# ---------------------------------------------------------------------------
# Kelly
# ---------------------------------------------------------------------------
class TestKelly:
    def test_kelly_zero_when_b_zero(self) -> None:
        assert RiskEngine.kelly(0.7, 0.0) == 0.0

    def test_kelly_zero_when_no_edge(self) -> None:
        # p*(b+1)-1 < 0  -> truncado a 0
        assert RiskEngine.kelly(0.4, 1.0) == 0.0

    def test_kelly_classic_50_50(self) -> None:
        # p=0.6 b=1 → f* = (0.6*2 - 1)/1 = 0.2
        assert RiskEngine.kelly(0.6, 1.0) == pytest.approx(0.2, abs=1e-9)

    def test_kelly_capped_at_one(self) -> None:
        assert RiskEngine.kelly(0.99, 0.5) <= 1.0


# ---------------------------------------------------------------------------
# Sizing y caps
# ---------------------------------------------------------------------------
class TestSizingAndCaps:
    def test_sizing_zero_if_gate_rejected(self) -> None:
        eng = RiskEngine()
        eng.update_balance(100_000_00)  # 100k USD en ¢
        s = eng.size(_gate(accepted=False), _readout(), "DUMMY")
        assert s.contracts == 0
        assert "gate" in s.rationale

    def test_sizing_respects_max_position_pct(self) -> None:
        # Con balance 1.000.000¢ y max_position_pct=0.05 →
        # exposición máxima 50.000¢ → al precio 50¢ ⇒ 1000 contratos
        eng = RiskEngine(RiskLimits(
            kelly_fraction=1.0, max_position_pct=0.05,
            max_market_exposure_pct=0.10, max_total_exposure_pct=0.50,
        ))
        eng.update_balance(1_000_000)
        s = eng.size(_gate(price_cents=50, edge_net=0.2),
                     _readout(p=0.99), "MKT")
        assert s.notional_cents <= int(0.05 * 1_000_000) + 50  # tolerancia 1 contrato
        assert s.contracts > 0

    def test_market_cap_blocks_repeats(self) -> None:
        eng = RiskEngine(RiskLimits(
            kelly_fraction=1.0, max_position_pct=0.50,
            max_market_exposure_pct=0.05, max_total_exposure_pct=0.50,
        ))
        eng.update_balance(1_000_000)
        s1 = eng.size(_gate(), _readout(p=0.95), "M1")
        eng.on_order("M1", s1.notional_cents)
        s2 = eng.size(_gate(), _readout(p=0.95), "M1")
        assert s2.notional_cents <= max(0, int(0.05 * 1_000_000) - s1.notional_cents) + 50

    def test_total_cap_blocks_global_exposure(self) -> None:
        eng = RiskEngine(RiskLimits(
            kelly_fraction=1.0, max_position_pct=0.50,
            max_market_exposure_pct=0.50, max_total_exposure_pct=0.10,
        ))
        eng.update_balance(1_000_000)
        eng.on_order("M1", 90_000)  # 9% ya colocado
        s = eng.size(_gate(), _readout(p=0.95), "M2")
        assert s.notional_cents <= 10_000 + 50


# ---------------------------------------------------------------------------
# Circuit breakers
# ---------------------------------------------------------------------------
class TestBreakers:
    def test_daily_dd_breaker(self) -> None:
        eng = RiskEngine(RiskLimits(daily_dd_limit_pct=0.05))
        eng.update_balance(1_000_000)
        eng.state.equity_high_day = 1_000_000
        # caída del 6%
        eng.state.balance_cents = 940_000
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert s.safe_mode is True
        assert any("daily_dd" in b for b in eng.state.breakers)

    def test_weekly_dd_breaker(self) -> None:
        eng = RiskEngine(RiskLimits(weekly_dd_limit_pct=0.10,
                                    daily_dd_limit_pct=0.50))
        eng.update_balance(1_000_000)
        eng.state.equity_high_week = 1_000_000
        eng.state.balance_cents = 800_000
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert s.safe_mode is True

    def test_consecutive_losses_breaker(self) -> None:
        eng = RiskEngine(RiskLimits(max_consecutive_losses=3))
        eng.update_balance(1_000_000)
        eng.state.consecutive_losses = 3
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert any("losses" in b for b in eng.state.breakers)

    def test_max_open_positions_breaker(self) -> None:
        eng = RiskEngine(RiskLimits(max_open_positions=2))
        eng.update_balance(1_000_000)
        eng.state.open_positions = {"A": 100, "B": 100}
        s = eng.size(_gate(), _readout(), "C")
        assert s.contracts == 0

    def test_rate_limit_breaker(self) -> None:
        eng = RiskEngine(RiskLimits(max_orders_per_minute=2))
        eng.update_balance(1_000_000)
        now = time.time()
        eng.state.order_times.append(now)
        eng.state.order_times.append(now)
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert any("rate_limit" in b for b in eng.state.breakers)


# ---------------------------------------------------------------------------
# Kill-switch
# ---------------------------------------------------------------------------
class TestKillSwitch:
    def test_kill_manual(self) -> None:
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        eng.kill("test")
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert s.safe_mode is True
        assert eng.state.kill_switch is True

    def test_reset_kill(self) -> None:
        eng = RiskEngine()
        eng.kill("test")
        eng.reset_kill()
        assert eng.state.kill_switch is False

    def test_kill_via_env(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("ATLAS_RADAR_KILL", "1")
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        s = eng.size(_gate(), _readout(), "M1")
        assert s.contracts == 0
        assert s.safe_mode is True


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------
class TestLifecycle:
    def test_on_close_resets_consecutive_on_win(self) -> None:
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        eng.state.consecutive_losses = 4
        eng.on_close("M1", 100, pnl_cents=500)
        assert eng.state.consecutive_losses == 0

    def test_on_close_increments_consecutive_on_loss(self) -> None:
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        eng.on_close("M1", 100, pnl_cents=-500)
        assert eng.state.consecutive_losses == 1

    def test_roll_day_clears_breakers(self) -> None:
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        eng.state.safe_mode = True
        eng.state.breakers.append("daily_dd=0.10")
        eng.roll_day()
        assert eng.state.safe_mode is False
        assert eng.state.breakers == []

    def test_status_serializable(self) -> None:
        eng = RiskEngine()
        eng.update_balance(1_000_000)
        st = eng.status()
        assert "safe_mode" in st
        assert "kill_switch" in st
        assert "exposure_cents" in st
        assert "limits" in st
