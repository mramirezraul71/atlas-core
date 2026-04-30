"""Tests para options.session_briefing.SessionBriefingEngine."""
from __future__ import annotations

import math
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.options.session_briefing import (
    SessionBriefingEngine,
    derive_dte_mode_from_days_to_event,
    strategy_to_recommended_family,
)


def _iv_ok(spot: float = 450.0, iv_rank: float = 40.0, quality: str = "ok"):
    return {
        "symbol": "SPY",
        "iv_current": 0.20,
        "iv_rank": iv_rank,
        "iv_hv_ratio": 1.05,
        "quality": quality,
        "spot": spot,
        "method": "test",
        "expiration": "2026-06-19",
        "dte": 30,
    }


class TestSessionBriefingEngine:
    def test_full_briefing_expected_move_and_strategy(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok(450.0, 40.0, "ok")
        mock_pick = MagicMock(return_value="iron_condor")
        engine = SessionBriefingEngine(iv, pick_strategy_fn=mock_pick)
        b = engine.build_briefing(
            "SPY",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
        )
        assert b["symbol"] == "SPY"
        assert b["gamma_regime"] == "long_gamma"
        assert b["dte_mode"] == "8to21"
        assert b["iv_rank"] == 40.0
        assert b["recommended_strategy"] == "iron_condor"
        assert b["recommended_family"] == "credit_neutral"
        assert b["expected_move"] is not None
        assert b["expected_move_range"] is not None
        horizon = 14  # 8to21 → _effective_dte_days_for_mode
        expected = 450.0 * 0.20 * math.sqrt(horizon / 252.0)
        assert pytest.approx(b["expected_move"], rel=1e-5) == expected
        assert b["expected_move_range"]["lower"] == pytest.approx(450.0 - expected, rel=1e-5)
        assert b["expected_move_range"]["upper"] == pytest.approx(450.0 + expected, rel=1e-5)
        mock_pick.assert_called_once()
        call_kw = mock_pick.call_args
        assert call_kw[0][0] == "BUY"
        assert call_kw[0][1] == "SIDEWAYS"
        assert call_kw[1]["gamma_regime"] == "long_gamma"
        assert call_kw[1]["dte_mode"] == "8to21"
        assert "recommended_strategy" in b
        assert b["recommended_strategy"] in b["strategy_candidates"]

    def test_no_gamma_sets_flag(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "bull_put_credit_spread")
        b = engine.build_briefing("SPY", direction="bullish", regime="bull", dte_mode="22plus")
        assert b["gamma_regime"] == "unknown"
        assert "no_gamma_regime" in b["quality_flags"]

    def test_levels_included_sorted_when_provided(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        b = engine.build_briefing(
            "SPY",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            support_levels=[450, 440.0, 445, 445],
            resistance_levels=[455.0, 470, 460, 460],
        )
        assert b["support_levels"] == [440.0, 445.0, 450.0]
        assert b["resistance_levels"] == [455.0, 460.0, 470.0]
        assert "structural_levels_provided_externally" in b["operational_notes"]

    def test_missing_levels_sets_flag_and_note(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        b = engine.build_briefing(
            "SPY",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
        )
        assert b["support_levels"] == []
        assert b["resistance_levels"] == []
        assert "no_structural_levels_provided" in b["operational_notes"]
        assert "no_structural_levels_provided" in b["quality_flags"]

    def test_event_near_sets_event_risk_flag(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        b = engine.build_briefing(
            "SPY",
            direction="neutral",
            regime="ranging",
            gamma_regime="short_gamma",
            dte_mode="0dte",
            event_near=True,
        )
        assert b["event_risk_flag"] is True
        assert "event_risk_flag_true_tighter_controls_suggested" in b["operational_notes"]

    def test_iv_quality_not_ok_flag(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok(quality="approx")
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        b = engine.build_briefing(
            "SPY",
            direction="neutral",
            regime="ranging",
            gamma_regime="short_gamma",
            dte_mode="1to7",
        )
        assert "iv_rank_quality_approx" in b["quality_flags"]
        assert "approx_iv_rank" in b["quality_flags"]

    def test_recommended_strategy_comes_from_pick_fn(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        sentinel = MagicMock(return_value="bear_call_credit_spread")
        engine = SessionBriefingEngine(iv, pick_strategy_fn=sentinel)
        b = engine.build_briefing("QQQ", direction="bearish", regime="bear", dte_mode="0dte")
        assert b["recommended_strategy"] == "bear_call_credit_spread"
        assert b["recommended_strategy"] in b["strategy_candidates"]
        sentinel.assert_called()
        assert sentinel.call_args[0][2] == 40.0  # iv_rank

    def test_derive_dte_mode_from_days(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        b = engine.build_briefing(
            "SPX",
            direction="neutral",
            regime="ranging",
            days_to_event=3,
        )
        assert b["dte_mode"] == "1to7"
        assert b["dte_mode_derived"] is True
        assert "dte_mode_derived_from_days_to_event" in b["quality_flags"]


class TestHelpers:
    def test_derive_dte_mode_from_days_to_event(self):
        assert derive_dte_mode_from_days_to_event(None) is None
        assert derive_dte_mode_from_days_to_event(0) == "0dte"
        assert derive_dte_mode_from_days_to_event(5) == "1to7"
        assert derive_dte_mode_from_days_to_event(15) == "8to21"
        assert derive_dte_mode_from_days_to_event(30) == "22plus"

    def test_strategy_to_recommended_family(self):
        assert strategy_to_recommended_family("iron_condor") == "credit_neutral"
        assert strategy_to_recommended_family("bull_put_credit_spread") == "credit_directional"
