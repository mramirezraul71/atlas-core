"""Sentinelas explícitos Options Engine (gauges + snapshot runtime)."""
from __future__ import annotations

import time

import pytest

from atlas_code_quant.options import options_engine_metrics as oem


def _reset_pipeline_ts(now: float) -> None:
    for m in oem._PIPELINE_MODULES:
        oem._state["module_last_ts"][m] = now


@pytest.fixture
def clean_sentinel_state(monkeypatch: pytest.MonkeyPatch):
    """Estado mínimo para sentinels sin arrastrar ejecuciones previas."""
    now = time.time()
    _reset_pipeline_ts(now)
    oem._state["last_journal_write_ts"] = now
    oem._state["last_iv_rank_quality_score"] = 1.0
    oem._state["paper_open_trades"] = 0
    oem._state["last_options_flow"] = {"coherent": True, "symbol": "SPX"}
    oem._state["module_last_ts"]["autoclose"] = now
    monkeypatch.delenv("ATLAS_OPTIONS_FLOW_BRIDGE", raising=False)
    yield now


def test_sentinel_healthy_snapshot(clean_sentinel_state, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.setenv("ATLAS_OPTIONS_FLOW_BRIDGE", "0")
    oem.update_options_engine_sentinels()
    snap = oem.get_last_sentinel_snapshot()
    assert snap is not None
    assert snap["metrics_freshness"] == pytest.approx(1.0)
    assert snap["journal_heartbeat"] == pytest.approx(1.0)
    assert snap["iv_rank_quality"] == pytest.approx(1.0)
    assert snap["options_flow"] == pytest.approx(0.25)
    assert snap["autoclose_activity"] >= 0.75


def test_sentinel_metrics_freshness_degraded_when_stale(clean_sentinel_state):
    pytest.importorskip("prometheus_client")
    old = time.time() - 4000.0
    for m in ("briefing", "intent_router", "entry_planner", "journal"):
        oem._state["module_last_ts"][m] = old
    oem.update_options_engine_sentinels(now=time.time())
    assert oem.get_last_sentinel_snapshot()["metrics_freshness"] == pytest.approx(0.25)


def test_sentinel_journal_degraded_when_old_write(clean_sentinel_state):
    pytest.importorskip("prometheus_client")
    oem._state["last_journal_write_ts"] = time.time() - 5000.0
    oem.update_options_engine_sentinels()
    hb = oem.get_last_sentinel_snapshot()["journal_heartbeat"]
    assert hb == pytest.approx(0.33)


def test_sentinel_iv_rank_degraded(clean_sentinel_state):
    pytest.importorskip("prometheus_client")
    oem._state["last_iv_rank_quality_score"] = 0.4
    oem.update_options_engine_sentinels()
    assert oem.get_last_sentinel_snapshot()["iv_rank_quality"] == pytest.approx(0.33)


def test_sentinel_options_flow_bridge_on_incoherent(clean_sentinel_state, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.setenv("ATLAS_OPTIONS_FLOW_BRIDGE", "1")
    oem._state["last_options_flow"] = {"coherent": False, "symbol": "SPX"}
    oem.update_options_engine_sentinels()
    assert oem.get_last_sentinel_snapshot()["options_flow"] == pytest.approx(0.0)


def test_sentinel_options_flow_bridge_on_coherent(clean_sentinel_state, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.setenv("ATLAS_OPTIONS_FLOW_BRIDGE", "1")
    oem._state["last_options_flow"] = {"coherent": True, "symbol": "SPX"}
    oem.update_options_engine_sentinels()
    assert oem.get_last_sentinel_snapshot()["options_flow"] == pytest.approx(1.0)


def test_sentinel_gauges_persist_in_registry(clean_sentinel_state, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    from prometheus_client import REGISTRY

    monkeypatch.setenv("ATLAS_OPTIONS_FLOW_BRIDGE", "0")
    oem.update_options_engine_sentinels()
    assert REGISTRY.get_sample_value("atlas_options_sentinel_metrics_freshness") == pytest.approx(1.0)
    assert REGISTRY.get_sample_value("atlas_options_sentinel_iv_rank_quality") == pytest.approx(1.0)


def test_record_session_plan_refreshes_sentinels(monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("prometheus_client")
    monkeypatch.delenv("ATLAS_OPTIONS_FLOW_BRIDGE", raising=False)
    plan = {
        "automation_mode": "paper_only",
        "symbol": "SPX",
        "entry_allowed": False,
        "briefing": {"iv_rank_payload_quality": "ok", "quality_flags": [], "iv_source": {"quality": "ok"}},
        "intent": {"allow_entry": False, "force_no_trade": True},
        "entry_plan": {"entry": "blocked"},
        "pipeline_notes": [],
        "pipeline_quality_flags": [],
    }
    oem.record_session_plan(plan, journal_path=None)
    snap = oem.get_last_sentinel_snapshot()
    assert snap is not None
    assert "metrics_freshness" in snap
