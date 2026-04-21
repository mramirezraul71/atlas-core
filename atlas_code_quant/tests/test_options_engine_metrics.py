"""Tests mínimos de métricas / snapshot Options Engine (paper)."""
from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.options import options_engine_metrics as oem
from atlas_code_quant.options.options_intent_router import OptionsIntentRouter
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
from atlas_code_quant.options.paper_session_orchestrator import PaperSessionOrchestrator
from atlas_code_quant.options.session_briefing import SessionBriefingEngine


def _iv_ok(spot: float = 5200.0, iv_rank: float = 40.0) -> dict:
    return {
        "symbol": "SPX",
        "iv_current": 0.18,
        "iv_rank": iv_rank,
        "iv_hv_ratio": 1.05,
        "quality": "ok",
        "spot": spot,
        "method": "test",
        "expiration": "2026-06-19",
        "dte": 30,
    }


def test_get_ui_snapshot_ok_keys():
    snap = oem.get_ui_snapshot()
    assert snap.get("ok") is True
    assert snap.get("automation_mode") == "paper_only"
    assert "go_nogo_label" in snap
    assert "grafana_health_dashboard_url" in snap
    assert "grafana_signals_intent_dashboard_url" in snap
    assert "grafana_paper_performance_dashboard_url" in snap
    assert "iv_rank_quality_score" in snap
    assert "iv_rank_quality_tier" in snap
    assert "options_self_audit" in snap
    assert "sentinel_snapshot" in snap
    assert "paper_performance_summary" in snap
    assert "options_engine" in snap
    assert "status" in (snap.get("options_engine") or {})


def test_get_ui_snapshot_sentinel_matches_get_last_sentinel_snapshot():
    pytest.importorskip("prometheus_client")
    oem.record_session_plan(
        {
            "automation_mode": "paper_only",
            "symbol": "SPX",
            "entry_allowed": False,
            "briefing": {"iv_rank_payload_quality": "ok", "quality_flags": [], "iv_source": {"quality": "ok"}},
            "intent": {"allow_entry": False, "force_no_trade": True},
            "entry_plan": {"entry": "blocked"},
            "pipeline_notes": [],
            "pipeline_quality_flags": [],
        },
        journal_path=None,
    )
    snap = oem.get_ui_snapshot()
    assert snap.get("sentinel_snapshot") == oem.get_last_sentinel_snapshot()


def test_get_ui_snapshot_win_rate_matches_paper_performance(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.chdir(tmp_path)
    jpath = tmp_path / "pj.jsonl"
    jpath.write_text(
        '{"event_type":"close_execution","trace_id":"t1","timestamp":"2026-04-01T12:00:00Z",'
        '"payload":{"pnl_realized": 50}}\n'
        '{"event_type":"close_execution","trace_id":"t2","timestamp":"2026-04-02T12:00:00Z",'
        '"payload":{"pnl_realized": -10}}\n'
    )
    oem._state["last_journal_path"] = str(jpath)
    snap = oem.get_ui_snapshot()
    perf = oem.get_last_paper_performance()
    assert perf is not None
    assert snap.get("win_rate_approx") == pytest.approx(float(perf["win_rate_ratio"]))
    assert snap.get("paper_performance_summary") is not None
    assert snap["paper_performance_summary"]["win_rate_ratio"] == pytest.approx(float(perf["win_rate_ratio"]))
    oe = snap.get("options_engine") or {}
    assert oe.get("trades_target") == 100
    assert "status" in oe
    assert "trades_closed_today" in oe


def test_iv_rank_quality_ok_no_flags():
    b = {"iv_rank_payload_quality": "ok", "quality_flags": [], "iv_source": {"quality": "ok"}}
    assert oem.compute_iv_rank_quality_score(b) == pytest.approx(1.0)


def test_iv_rank_quality_ok_with_iv_degrade_flags():
    b = {
        "iv_rank_payload_quality": "ok",
        "quality_flags": ["approx_iv_rank"],
        "iv_source": {"quality": "ok"},
    }
    assert oem.compute_iv_rank_quality_score(b) == pytest.approx(0.5)


def test_iv_rank_quality_approx():
    b = {"iv_rank_payload_quality": "approx", "quality_flags": [], "iv_source": {"quality": "approx"}}
    assert oem.compute_iv_rank_quality_score(b) == pytest.approx(0.5)


def test_iv_rank_quality_insufficient_history():
    b = {"iv_rank_payload_quality": "insufficient_history", "quality_flags": []}
    assert oem.compute_iv_rank_quality_score(b) == pytest.approx(0.5)


def test_iv_rank_quality_missing_and_empty_briefing():
    assert oem.compute_iv_rank_quality_score(None) == pytest.approx(0.25)
    assert oem.compute_iv_rank_quality_score({}) == pytest.approx(0.25)
    assert oem.compute_iv_rank_quality_score({"quality_flags": []}) == pytest.approx(0.25)


def test_iv_rank_quality_iv_source_error():
    b = {"iv_rank_payload_quality": "ok", "iv_source": {"quality": "ok", "error": "upstream_failed"}}
    assert oem.compute_iv_rank_quality_score(b) == pytest.approx(0.0)


def test_record_session_plan_sets_last_iv_score():
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY

    plan = {
        "automation_mode": "paper_only",
        "symbol": "SPX",
        "entry_allowed": True,
        "briefing": {"iv_rank_payload_quality": "ok", "quality_flags": [], "iv_source": {"quality": "ok"}, "iv_rank": 42.0},
        "intent": {"allow_entry": True, "force_no_trade": False},
        "entry_plan": {"entry": "proposed"},
        "pipeline_notes": [],
        "pipeline_quality_flags": [],
    }
    oem.record_session_plan(plan, journal_path=None)
    assert oem.get_last_iv_rank_quality_score() == pytest.approx(1.0)
    assert REGISTRY.get_sample_value("atlas_options_iv_rank_quality") == pytest.approx(2.0)
    assert REGISTRY.get_sample_value("atlas_options_iv_rank_value") == pytest.approx(42.0)


def test_orchestrator_updates_metrics_without_error(tmp_path: Path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    jpath = tmp_path / "data" / "journal.jsonl"
    jpath.parent.mkdir(parents=True, exist_ok=True)
    from atlas_code_quant.options.options_paper_journal import OptionsPaperJournal

    journal = OptionsPaperJournal(path=jpath)
    iv = MagicMock()
    iv.get_iv_rank.return_value = _iv_ok()
    engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
    orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner(), journal=journal)
    out = orch.build_session_plan(
        symbol="SPX",
        direction="neutral",
        regime="ranging",
        gamma_regime="long_gamma",
        dte_mode="8to21",
        capital=10_000.0,
    )
    assert out["symbol"] == "SPX"
    vs = out["briefing"].get("visual_signal") or {}
    assert vs.get("availability") == "ok"
    assert vs.get("source") == "visual_signal_adapter"
    snap = oem.get_ui_snapshot()
    assert snap.get("ok") is True
    assert snap.get("symbol") == "SPX"
    assert snap.get("iv_rank_quality_score") == pytest.approx(1.0)


def test_compute_options_self_audit_state_score_mapping():
    assert oem.compute_options_self_audit_state_score(None) == pytest.approx(0.1)
    assert oem.compute_options_self_audit_state_score({"status": "error"}) == pytest.approx(0.0)
    assert oem.compute_options_self_audit_state_score({"status": "skipped"}) == pytest.approx(0.25)
    assert oem.compute_options_self_audit_state_score(
        {"status": "ok", "passed": True, "overall_severity": "INFO"}
    ) == pytest.approx(1.0)
    assert oem.compute_options_self_audit_state_score(
        {"status": "ok", "passed": True, "overall_severity": "WARN"}
    ) == pytest.approx(0.72)
    assert oem.compute_options_self_audit_state_score(
        {"status": "ok", "passed": False, "overall_severity": "BLOCK"}
    ) == pytest.approx(0.35)


def test_record_session_plan_persists_self_audit_metric():
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY

    plan = {
        "automation_mode": "paper_only",
        "symbol": "SPX",
        "entry_allowed": False,
        "briefing": {},
        "intent": {"allow_entry": False, "force_no_trade": True},
        "entry_plan": {"entry": "blocked"},
        "pipeline_notes": [],
        "pipeline_quality_flags": [],
        "options_self_audit": {
            "status": "ok",
            "source": "operational_self_audit",
            "timestamp": "2026-01-01T00:00:00+00:00",
            "passed": True,
            "overall_severity": "WARN",
            "findings_count": 2,
            "blocking_findings": 0,
        },
    }
    oem.record_session_plan(plan, journal_path=None)
    st = oem.get_last_options_self_audit()
    assert st is not None
    assert st.get("status") == "ok"
    assert st.get("findings_count") == 2
    assert REGISTRY.get_sample_value("atlas_options_self_audit_state") == pytest.approx(0.72)


def test_compute_visual_signal_state_score_mapping():
    assert oem.compute_visual_signal_state_score(None) == pytest.approx(0.2)
    assert oem.compute_visual_signal_state_score({"availability": "error"}) == pytest.approx(0.0)
    assert oem.compute_visual_signal_state_score({"availability": "unavailable"}) == pytest.approx(0.2)
    assert oem.compute_visual_signal_state_score({"availability": "ok"}) == pytest.approx(0.5)
    assert oem.compute_visual_signal_state_score(
        {"availability": "ok", "breach_detected": True, "critical_breach": False}
    ) == pytest.approx(0.75)
    assert oem.compute_visual_signal_state_score(
        {"availability": "ok", "breach_detected": True, "critical_breach": True}
    ) == pytest.approx(1.0)


def test_record_session_plan_persists_visual_signal_state_metric():
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY

    plan = {
        "automation_mode": "paper_only",
        "symbol": "SPX",
        "entry_allowed": False,
        "briefing": {
            "visual_signal": {
                "availability": "ok",
                "breach_detected": False,
                "critical_breach": False,
                "source": "visual_signal_adapter",
            }
        },
        "intent": {"allow_entry": False, "force_no_trade": True},
        "entry_plan": {"entry": "blocked"},
        "pipeline_notes": [],
        "pipeline_quality_flags": [],
    }
    oem.record_session_plan(plan, journal_path=None)
    assert oem.get_last_visual_signal() is not None
    assert oem.get_last_visual_signal().get("availability") == "ok"
    assert REGISTRY.get_sample_value("atlas_options_visual_signal_state") == pytest.approx(0.5)


def _flow_ok(symbol: str = "SPX") -> dict:
    return {
        "available": True,
        "symbol": symbol,
        "put_call_volume_ratio": 1.12,
        "gamma_bias_pct": 3.5,
        "score_pct": 62.0,
        "confidence_pct": 71.0,
        "mode": "options_flow",
    }


def test_apply_options_flow_valid_snapshot():
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY

    row = oem.apply_options_flow_snapshot_to_metrics("SPX", _flow_ok())
    assert row["coherent"] is True
    assert row["payload_available"] == pytest.approx(1.0)
    assert row["put_call_volume_ratio"] == pytest.approx(1.12)
    st = oem.get_last_options_flow_metrics()
    assert st is not None
    assert st["score_pct"] == pytest.approx(62.0)
    assert REGISTRY.get_sample_value("atlas_options_flow_payload_available", {"symbol": "SPX"}) == pytest.approx(1.0)


def test_apply_options_flow_none_is_conservative():
    pytest.importorskip("prometheus_client")
    row = oem.apply_options_flow_snapshot_to_metrics("QQQ", None)
    assert row["coherent"] is False
    assert row["payload_available"] == pytest.approx(0.0)
    assert row["put_call_volume_ratio"] == pytest.approx(oem._FLOW_SENTINEL)
    assert row["confidence_pct"] == pytest.approx(0.0)
    st = oem.get_last_options_flow_metrics()
    assert st is not None
    assert st["symbol"] == "QQQ"


def test_apply_options_flow_incomplete_payload_not_invented():
    """``available`` true sin numéricos obligatorios → mismo tratamiento que ausente (no ratios falsos)."""
    pytest.importorskip("prometheus_client")
    row = oem.apply_options_flow_snapshot_to_metrics("SPX", {"available": True})
    assert row["coherent"] is False
    assert row["payload_available"] == pytest.approx(0.0)
    assert row["put_call_volume_ratio"] == pytest.approx(oem._FLOW_SENTINEL)


def test_record_session_plan_persists_options_flow_metrics():
    pytest.importorskip("prometheus_client")
    plan = {
        "automation_mode": "paper_only",
        "symbol": "SPX",
        "entry_allowed": False,
        "briefing": {},
        "intent": {"allow_entry": False, "force_no_trade": True},
        "entry_plan": {"entry": "blocked"},
        "pipeline_notes": [],
        "pipeline_quality_flags": [],
        "options_flow_snapshot": _flow_ok("SPX"),
    }
    oem.record_session_plan(plan, journal_path=None)
    st = oem.get_last_options_flow_metrics()
    assert st is not None
    assert st["coherent"] is True
    assert st["gamma_bias_pct"] == pytest.approx(3.5)


def test_compute_paper_performance_insufficient_for_win_rate():
    m = oem.compute_paper_performance_from_pnls([50.0])
    assert m["n_closes"] == 1
    assert m["win_rate_ratio"] == pytest.approx(-1.0)
    assert m["insufficient_win_rate_sample"] is True


def test_compute_paper_performance_wr_pf_net_drawdown():
    pnls = [100.0, -40.0, 60.0, -20.0]
    m = oem.compute_paper_performance_from_pnls(pnls)
    assert m["n_closes"] == 4
    assert m["win_rate_ratio"] == pytest.approx(0.5)
    assert m["profit_factor"] == pytest.approx(160.0 / 60.0, rel=1e-6)
    assert m["net_realized_pnl_usd"] == pytest.approx(100.0)
    assert m["max_drawdown_usd"] == pytest.approx(40.0)


def test_compute_paper_performance_profit_factor_capped_no_losses():
    m = oem.compute_paper_performance_from_pnls([10.0, 5.0])
    assert m["profit_factor"] == pytest.approx(99.0)
    assert m["insufficient_profit_factor_capped_no_losses"] is True
    assert m["win_rate_ratio"] == pytest.approx(1.0)


def test_compute_paper_performance_only_losses_pf_zero():
    m = oem.compute_paper_performance_from_pnls([-10.0, -5.0])
    assert m["profit_factor"] == pytest.approx(0.0)
    assert m["win_rate_ratio"] == pytest.approx(0.0)


def test_extract_close_realized_pnls_from_journal_text():
    text = (
        '{"event_type":"close_execution","trace_id":"a","timestamp":"2026-04-01T12:00:00Z",'
        '"payload":{"pnl_realized":100}}\n'
        '{"event_type":"noise","x":1}\n'
        '{"event_type":"close_execution","trace_id":"b","timestamp":"2026-04-02T12:00:00Z",'
        '"payload":{"pnl_realized":-40}}\n'
    )
    got = oem.extract_close_realized_pnls_from_journal_text(text)
    assert got == [100.0, -40.0]


def test_refresh_journal_from_disk_sets_paper_performance_state(tmp_path: Path, monkeypatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.chdir(tmp_path)
    jpath = tmp_path / "pj.jsonl"
    jpath.write_text(
        '{"event_type":"close_execution","trace_id":"t1","timestamp":"2026-04-10T12:00:00Z",'
        '"payload":{"pnl_realized":50}}\n'
        '{"event_type":"close_execution","trace_id":"t2","timestamp":"2026-04-11T12:00:00Z",'
        '"payload":{"pnl_realized":-10}}\n',
        encoding="utf-8",
    )
    oem.refresh_journal_from_disk(jpath)
    st = oem.get_last_paper_performance()
    assert st is not None
    assert st["n_closes"] == 2
    assert st["win_rate_ratio"] == pytest.approx(0.5)
    assert st["net_realized_pnl_usd"] == pytest.approx(40.0)


def test_refresh_journal_tracks_go_blocked_sessions_metrics(tmp_path: Path, monkeypatch):
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY
    from datetime import datetime, timezone

    monkeypatch.chdir(tmp_path)
    jpath = tmp_path / "pj_blocked.jsonl"
    now_iso = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    jpath.write_text(
        f'{{"event_type":"session_plan","trace_id":"t1","timestamp":"{now_iso}","payload":{{"session_plan":{{"entry_allowed":true,"intent":{{"force_no_trade":false}}}}}}}}\n'
        f'{{"event_type":"entry_blocked","trace_id":"t1","timestamp":"{now_iso}","entry_executable":false,"position_size_units":0}}\n',
        encoding="utf-8",
    )
    oem.refresh_journal_from_disk(jpath)
    assert REGISTRY.get_sample_value("atlas_options_paper_sessions_go_blocked_today") == pytest.approx(1.0)


def test_auto_close_metrics_hook_no_crash():
    """``evaluate_position`` llama a ``record_autoclose_triggers`` sin romper si Prometheus no está."""
    from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine

    eng = AutoCloseEngine()
    pos = {
        "position_id": "p1",
        "entry_credit": 1.0,
        "current_value": 0.3,
        "unrealized_pnl": 0.55,
        "remaining_dte": 30,
        "is_0dte": False,
        "is_credit": True,
        "strategy_type": "credit",
    }
    d = eng.evaluate_position(pos)
    assert d["should_close"] is True
    assert "take_profit" in d["reasons"]
