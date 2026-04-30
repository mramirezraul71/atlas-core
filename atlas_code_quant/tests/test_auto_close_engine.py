"""Tests para execution.auto_close_engine."""
from __future__ import annotations

from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine


class TestAutoCloseEngine:
    def test_credit_take_profit_triggers_close(self):
        engine = AutoCloseEngine(profit_take_pct_credit=0.50)
        position = {
            "position_id": "p1",
            "strategy_type": "bull_put_credit_spread",
            "entry_credit": 2.0,
            "current_value": 0.8,
            "remaining_dte": 28,
            "is_0dte": False,
            "is_credit": True,
        }
        decision = engine.evaluate_position(position)
        assert decision["should_close"] is True
        assert "take_profit" in decision["reasons"]
        assert decision["recommended_action"] == "close"

    def test_credit_stop_loss_triggers_close(self):
        engine = AutoCloseEngine(stop_loss_multiple_credit=2.0)
        position = {
            "position_id": "p2",
            "strategy_type": "bear_call_credit_spread",
            "entry_credit": 2.0,
            "current_value": 6.0,  # pérdida = -4.0 => 2x crédito
            "remaining_dte": 30,
            "is_credit": True,
        }
        decision = engine.evaluate_position(position)
        assert decision["should_close"] is True
        assert "stop_loss" in decision["reasons"]
        assert decision["priority"] == "high"

    def test_no_condition_keeps_position_open(self):
        engine = AutoCloseEngine(min_remaining_dte=5)
        position = {
            "position_id": "p3",
            "strategy_type": "bull_put_credit_spread",
            "entry_credit": 2.0,
            "current_value": 1.4,  # pnl=0.6, no TP(1.0), no SL
            "remaining_dte": 12,
            "is_credit": True,
            "is_0dte": False,
        }
        decision = engine.evaluate_position(position)
        assert decision["should_close"] is False
        assert decision["reasons"] == []
        assert decision["recommended_action"] == "hold"

    def test_dte_gate_triggers_for_non_0dte(self):
        engine = AutoCloseEngine(min_remaining_dte=21)
        position = {
            "position_id": "p4",
            "strategy_type": "iron_condor",
            "entry_credit": 1.5,
            "current_value": 1.4,
            "remaining_dte": 10,
            "is_credit": True,
            "is_0dte": False,
        }
        decision = engine.evaluate_position(position)
        assert decision["should_close"] is True
        assert "dte_gate" in decision["reasons"]

    def test_0dte_hook_triggers_only_when_context_present(self):
        engine = AutoCloseEngine(enable_0dte_breach_hook=True)
        base_position = {
            "position_id": "p5",
            "strategy_type": "iron_condor",
            "entry_credit": 1.5,
            "current_value": 1.6,
            "remaining_dte": 0,
            "is_credit": True,
            "is_0dte": True,
        }
        without_ctx = engine.evaluate_position(base_position)
        assert "0dte_critical_breach" not in without_ctx["reasons"]

        with_ctx = engine.evaluate_position(
            base_position,
            breach_context={"short_strike_breached": True},
        )
        assert with_ctx["should_close"] is True
        assert "0dte_critical_breach" in with_ctx["reasons"]
        assert with_ctx["priority"] == "high"

    def test_scan_positions_returns_decisions_for_all(self):
        engine = AutoCloseEngine()
        positions = [
            {
                "position_id": "a",
                "strategy_type": "bull_put_credit_spread",
                "entry_credit": 2.0,
                "current_value": 0.9,
                "remaining_dte": 30,
                "is_credit": True,
            },
            {
                "position_id": "b",
                "strategy_type": "bear_call_credit_spread",
                "entry_credit": 2.0,
                "current_value": 6.0,
                "remaining_dte": 30,
                "is_credit": True,
            },
            {
                "position_id": "c",
                "strategy_type": "long_call",
                "current_value": 3.0,
                "remaining_dte": 40,
                "is_credit": False,
            },
        ]
        decisions = engine.scan_positions(positions)
        assert len(decisions) == 3
        assert {d["position_id"] for d in decisions} == {"a", "b", "c"}
        assert all("should_close" in d for d in decisions)
