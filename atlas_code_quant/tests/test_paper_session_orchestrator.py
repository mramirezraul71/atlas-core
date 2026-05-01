"""Tests para paper session orchestrator y endpoint (solo paper, sin órdenes)."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.operations.operational_self_audit import SelfAuditCheck, SelfAuditResult
from atlas_code_quant.options import options_engine_metrics as oem
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_code_quant.options.options_intent_router import OptionsIntentRouter
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
from atlas_code_quant.options.paper_session_orchestrator import PaperSessionOrchestrator
from atlas_code_quant.options.session_briefing import SessionBriefingEngine

pytest.importorskip("fastapi")


def _iv_ok(spot: float = 5200.0, iv_rank: float = 40.0, quality: str = "ok") -> dict:
    return {
        "symbol": "SPX",
        "iv_current": 0.18,
        "iv_rank": iv_rank,
        "iv_hv_ratio": 1.05,
        "quality": quality,
        "spot": spot,
        "method": "test",
        "expiration": "2026-06-19",
        "dte": 30,
    }


def _rich_briefing(**overrides) -> dict:
    base = {
        "symbol": "SPX",
        "direction": "neutral",
        "regime": "ranging",
        "gamma_regime": "long_gamma",
        "dte_mode": "8to21",
        "event_risk_flag": False,
        "market_bias": "neutral_mean_reversion",
        "risk_posture": "balanced",
        "recommended_strategy": "iron_condor",
        "recommended_family": "credit_neutral",
        "strategy_candidates": ["iron_condor", "bull_put_credit_spread"],
        "visual_breach_flag": False,
        "breach_context": {},
        "quality_flags": [],
        "operational_notes": [],
    }
    base.update(overrides)
    return base


class TestPaperSessionOrchestrator:
    def test_base_valid_pipeline(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            capital=25_000.0,
        )
        assert out["automation_mode"] == "paper_only"
        assert out["symbol"] == "SPX"
        assert out["briefing"].get("symbol") == "SPX"
        assert out["intent"]["symbol"] == "SPX"
        assert out["entry_plan"]["mode"] == "paper_only"
        assert out["entry_plan"]["entry"] == "proposed"
        assert out["entry_allowed"] is True
        assert out["entry_plan"]["max_risk_budget_dollars"] == pytest.approx(500.0)

    def test_force_no_trade_yields_no_entry(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            manual_intent_overrides={"force_no_trade": True, "allow_entry": False},
        )
        assert out["intent"]["force_no_trade"] is True
        assert out["entry_plan"]["entry"] == "none"
        assert out["entry_plan"]["reason"] == "force_no_trade"
        assert out["entry_allowed"] is False

    def test_quality_flags_propagate(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPY",
            direction="bullish",
            regime="bull",
            dte_mode="22plus",
        )
        assert "no_gamma_regime" in (out["briefing"].get("quality_flags") or [])
        assert any(f.startswith("briefing:") for f in out["pipeline_quality_flags"])

    def test_breach_context_0dte_blocks_entry(self):
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="0dte",
            spot=5200.0,
            breach_context={"breach_detected": True, "source": "visual"},
        )
        assert out["intent"]["allow_entry"] is False
        assert out["entry_plan"]["entry"] == "none"
        assert out["entry_allowed"] is False


class TestOptionsSelfAuditHook:
    """Self-audit operativo en ``build_session_plan`` (observacional, no bloquea)."""

    def test_self_audit_present_after_real_run(self, monkeypatch, tmp_path):
        monkeypatch.chdir(tmp_path)
        monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            capital=25_000.0,
        )
        sa = out.get("options_self_audit")
        assert isinstance(sa, dict)
        assert sa.get("source") == "operational_self_audit"
        assert sa.get("status") == "ok"
        assert "timestamp" in sa
        assert "findings_count" in sa
        assert "blocking_findings" in sa
        assert oem.get_last_options_self_audit() is not None
        assert oem.get_last_options_self_audit().get("status") == "ok"

    def test_self_audit_skipped_when_disabled(self, monkeypatch, tmp_path):
        monkeypatch.chdir(tmp_path)
        monkeypatch.setenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", "0")
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
        )
        sa = out.get("options_self_audit")
        assert sa.get("status") == "skipped"
        assert sa.get("passed") is None

    def test_self_audit_warn_findings(self, monkeypatch, tmp_path):
        monkeypatch.chdir(tmp_path)
        monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)

        def _fake_run(ctx):
            return SelfAuditResult(
                passed=True,
                overall_severity="WARN",
                checks=[
                    SelfAuditCheck(
                        id="risk.test",
                        category="risk",
                        severity="WARN",
                        message="warn",
                    )
                ],
                ts_utc="2026-04-01T12:00:00+00:00",
                scope="paper",
            )

        monkeypatch.setattr(
            "atlas_code_quant.options.paper_session_orchestrator.run_operational_self_audit",
            _fake_run,
        )
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
        )
        sa = out.get("options_self_audit")
        assert sa.get("status") == "ok"
        assert sa.get("overall_severity") == "WARN"
        assert sa.get("findings_count") == 1
        pytest.importorskip("prometheus_client")
        from prometheus_client import REGISTRY

        assert REGISTRY.get_sample_value("atlas_options_self_audit_state") == pytest.approx(0.72)

    def test_self_audit_exception_degraded(self, monkeypatch, tmp_path):
        monkeypatch.chdir(tmp_path)
        monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)

        def _boom(ctx):
            raise RuntimeError("audit failed")

        monkeypatch.setattr(
            "atlas_code_quant.options.paper_session_orchestrator.run_operational_self_audit",
            _boom,
        )
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok()
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
        )
        assert out.get("symbol") == "SPX"
        sa = out.get("options_self_audit")
        assert sa.get("status") == "error"
        assert "error" in sa
        assert oem.get_last_options_self_audit().get("status") == "error"
        pytest.importorskip("prometheus_client")
        from prometheus_client import REGISTRY

        assert REGISTRY.get_sample_value("atlas_options_self_audit_state") == pytest.approx(0.0)


class TestPaperSessionPlanEndpoint:
    def test_paper_session_plan_endpoint_ok(self):
        from atlas_code_quant.api.routes import options as options_routes

        briefing = _rich_briefing()

        app = FastAPI()
        app.include_router(options_routes.router)

        with patch(
            "atlas_code_quant.options.session_briefing.SessionBriefingEngine",
        ) as MockEng:
            MockEng.return_value.build_briefing.return_value = briefing
            with patch(
                "atlas_code_quant.backtesting.winning_probability.TradierClient",
            ) as MockTradier:
                MockTradier.return_value = MagicMock()
                with patch(
                    "atlas_code_quant.options.iv_rank_calculator.IVRankCalculator",
                ) as MockIv:
                    MockIv.return_value = MagicMock()
                    with TestClient(app) as client:
                        resp = client.post(
                            "/options/paper-session-plan",
                            json={
                                "symbol": "SPX",
                                "direction": "neutral",
                                "regime": "ranging",
                                "gamma_regime": "long_gamma",
                                "dte_mode": "8to21",
                                "capital": 25000,
                            },
                        )
        assert resp.status_code == 200, resp.text
        body = resp.json()
        assert body["ok"] is True
        data = body["data"]
        assert data["automation_mode"] == "paper_only"
        assert data["symbol"] == "SPX"
        assert "briefing" in data and "intent" in data and "entry_plan" in data
