"""Tests para options.options_intent_router."""
from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from atlas_code_quant.options.options_intent_router import OptionsIntentRouter

pytest.importorskip("fastapi")


def _base_briefing() -> dict:
    return {
        "symbol": "SPX",
        "direction": "neutral",
        "regime": "ranging",
        "gamma_regime": "long_gamma",
        "dte_mode": "0dte",
        "event_risk_flag": False,
        "market_bias": "neutral_mean_reversion",
        "risk_posture": "high_caution_0dte",
        "recommended_strategy": "iron_condor",
        "recommended_family": "credit_neutral",
        "strategy_candidates": ["iron_condor", "bull_put_credit_spread", "bear_call_credit_spread"],
        "visual_breach_flag": False,
        "breach_context": {},
    }


class TestOptionsIntentRouter:
    def test_base_long_gamma_ranging_prefers_credit_families(self):
        router = OptionsIntentRouter()
        intent = router.build_intent(
            {
                **_base_briefing(),
                "dte_mode": "8to21",
                "risk_posture": "balanced",
            }
        )
        assert intent["allow_entry"] is True
        assert "credit_neutral" in intent["preferred_families"]
        assert intent["force_no_trade"] is False

    def test_short_gamma_0dte_suppresses_short_premium(self):
        router = OptionsIntentRouter()
        briefing = {
            **_base_briefing(),
            "direction": "bullish",
            "regime": "trend",
            "gamma_regime": "short_gamma",
            "market_bias": "bullish",
            "recommended_strategy": "bull_call_debit_spread",
            "recommended_family": "debit_directional",
            "strategy_candidates": ["bull_call_debit_spread", "long_call", "iron_condor"],
        }
        intent = router.build_intent(briefing)
        assert intent["suppress_short_premium"] is True
        assert "debit_directional" in intent["preferred_families"]
        assert "long_premium" in intent["preferred_families"]

    def test_visual_breach_0dte_restricts_entry(self):
        router = OptionsIntentRouter()
        briefing = {
            **_base_briefing(),
            "visual_breach_flag": True,
            "event_risk_flag": False,
            "risk_posture": "high_caution_0dte",
        }
        intent = router.build_intent(briefing)
        assert intent["allow_entry"] is False
        assert "visual_breach_detected_reduce_aggression" in intent["intent_notes"]

    def test_event_risk_plus_visual_breach_forces_no_trade(self):
        router = OptionsIntentRouter()
        briefing = {
            **_base_briefing(),
            "visual_breach_flag": True,
            "event_risk_flag": True,
            "risk_posture": "high_caution_0dte",
        }
        intent = router.build_intent(briefing)
        assert intent["force_no_trade"] is True
        assert intent["allow_entry"] is False

    def test_manual_overrides_applied(self):
        router = OptionsIntentRouter()
        intent = router.build_intent(
            _base_briefing(),
            manual_overrides={
                "allow_entry": False,
                "force_no_trade": True,
                "preferred_families": ["debit_directional"],
            },
        )
        assert intent["allow_entry"] is False
        assert intent["force_no_trade"] is True
        assert intent["preferred_families"] == ["debit_directional"]
        assert "manual_override_applied" in intent["intent_notes"]

    def test_strategy_candidates_filtered_when_family_blocked(self):
        router = OptionsIntentRouter()
        briefing = {
            **_base_briefing(),
            "visual_breach_flag": True,
            "event_risk_flag": False,
            "recommended_strategy": "iron_condor",  # credit_neutral
            "recommended_family": "credit_neutral",
            "strategy_candidates": ["iron_condor", "bull_call_debit_spread", "long_call"],
        }
        intent = router.build_intent(briefing)
        assert "credit_neutral" in intent["blocked_families"]
        assert "iron_condor" not in intent["strategy_candidates"]
        assert intent["recommended_strategy"] is None

    def test_quality_flags_missing_gamma_and_dte(self):
        router = OptionsIntentRouter()
        briefing = {
            "symbol": "SPY",
            "direction": "neutral",
            "regime": "ranging",
            "market_bias": "neutral",
            "risk_posture": "balanced",
            "recommended_strategy": "iron_condor",
            "strategy_candidates": ["iron_condor"],
            "event_risk_flag": False,
        }
        intent = router.build_intent(briefing)
        assert "missing_gamma_regime" in intent["intent_quality_flags"]
        assert "missing_dte_mode" in intent["intent_quality_flags"]

    def test_intent_endpoint_returns_payload(self):
        from atlas_code_quant.api.routes import options as options_routes

        app = FastAPI()
        app.include_router(options_routes.router)
        with TestClient(app) as client:
            resp = client.post(
                "/options/intent",
                json={"briefing": _base_briefing(), "manual_overrides": {"allow_entry": False}},
            )
        assert resp.status_code == 200
        body = resp.json()
        assert body["ok"] is True
        assert "data" in body
        assert body["data"]["symbol"] == "SPX"
