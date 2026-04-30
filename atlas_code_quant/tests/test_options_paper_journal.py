"""Tests del journal JSONL paper-only para opciones."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.options.options_paper_journal import OptionsPaperJournal
from atlas_code_quant.options.options_intent_router import OptionsIntentRouter
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
import atlas_code_quant.options.paper_session_orchestrator as orch_mod
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


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows: list[dict] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


class TestOptionsPaperJournal:
    def test_log_session_plan_creates_file_returns_trace(self, tmp_path: Path):
        p = tmp_path / "j.jsonl"
        j = OptionsPaperJournal(path=p)
        plan = {
            "automation_mode": "paper_only",
            "symbol": "SPX",
            "entry_allowed": True,
            "briefing": {"direction": "neutral", "gamma_regime": "long_gamma"},
            "intent": {"allow_entry": True, "force_no_trade": False},
            "entry_plan": {"entry": "proposed", "recommended_strategy": "iron_condor", "mode": "paper_only"},
            "pipeline_notes": [],
            "pipeline_quality_flags": [],
        }
        tid = j.log_session_plan(plan, notes=["paper_session_plan_auto"])
        assert tid
        assert p.is_file()
        rows = _read_jsonl(p)
        assert len(rows) == 1
        assert rows[0]["event_type"] == "session_plan"
        assert rows[0]["trace_id"] == tid
        assert rows[0]["symbol"] == "SPX"
        assert rows[0]["payload"]["session_plan"]["trace_id"] == tid
        assert rows[0]["payload"]["briefing_summary"]["gamma_regime"] == "long_gamma"
        assert rows[0]["payload"]["intent_summary"]["allow_entry"] is True
        assert rows[0]["payload"]["entry_plan_summary"]["entry"] == "proposed"
        assert rows[0]["notes"] == ["paper_session_plan_auto"]
        assert rows[0]["mode"] == "paper"
        assert rows[0]["source"] == "planner"
        assert rows[0]["journal_version"] == "1.0"
        assert rows[0]["timestamp_utc"].endswith("Z")
        assert rows[0]["status"] == "open"
        assert rows[0]["underlying"] == "SPX"

    def test_log_session_plan_respects_existing_trace_id(self, tmp_path: Path):
        p = tmp_path / "j2.jsonl"
        j = OptionsPaperJournal(path=p)
        tid = j.log_session_plan({"symbol": "QQQ", "trace_id": "custom-1", "briefing": {}, "intent": {}, "entry_plan": {}})
        assert tid == "custom-1"

    def test_log_entry_execution(self, tmp_path: Path):
        p = tmp_path / "e.jsonl"
        j = OptionsPaperJournal(path=p)
        ts = datetime(2026, 4, 21, 14, 30, tzinfo=timezone.utc)
        j.log_entry_execution(
            trace_id="t1",
            symbol="SPX",
            entry_timestamp=ts,
            planned_entry={"entry": "proposed"},
            executed_entry={"strikes": [5000, 5010], "credit": 1.2},
            notes=["manual_paper"],
        )
        rows = _read_jsonl(p)
        assert len(rows) == 1
        assert rows[0]["event_type"] == "entry_execution"
        assert rows[0]["trace_id"] == "t1"
        assert rows[0]["symbol"] == "SPX"
        assert "2026-04-21" in rows[0]["timestamp"]
        assert rows[0]["payload"]["executed_entry"]["credit"] == 1.2
        assert rows[0]["entry_credit"] == pytest.approx(1.2)
        assert rows[0]["structure_type"] is None
        assert rows[0]["mode"] == "paper"
        assert rows[0]["status"] == "open"
        assert rows[0]["journal_version"] == "1.0"

    def test_log_close_decision_and_close_execution(self, tmp_path: Path):
        p = tmp_path / "c.jsonl"
        j = OptionsPaperJournal(path=p)
        ts = datetime(2026, 4, 22, 15, 0, tzinfo=timezone.utc)
        j.log_close_decision(
            trace_id="t2",
            close_decision={"symbol": "SPX", "action": "close_winners", "reason": "test"},
            timestamp=ts,
            notes=["auto_close_engine"],
        )
        j.log_close_execution(
            trace_id="t2",
            symbol="SPX",
            close_timestamp=ts,
            executed_close={"avg_price": 0.5},
            pnl_realized=120.5,
            strategy_type="iron_condor",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            notes=["paper_close"],
        )
        rows = _read_jsonl(p)
        assert len(rows) == 2
        assert rows[0]["event_type"] == "close_decision"
        assert rows[0]["payload"]["close_decision"]["action"] == "close_winners"
        assert rows[1]["event_type"] == "close_execution"
        assert rows[1]["payload"]["pnl_realized"] == pytest.approx(120.5)
        assert rows[1]["status"] == "closed"
        assert rows[1]["pnl_usd"] == pytest.approx(120.5)
        assert rows[1]["close_type"] == "full"
        assert rows[1]["strategy_type"] == "iron_condor"
        assert rows[1]["structure_type"] == "iron_condor"
        assert rows[1]["gamma_regime"] == "long_gamma"
        assert rows[1]["dte_mode"] == "8to21"
        assert rows[1]["journal_version"] == "1.0"

    def test_all_lines_valid_jsonl(self, tmp_path: Path):
        p = tmp_path / "all.jsonl"
        j = OptionsPaperJournal(path=p)
        base_ts = datetime(2026, 4, 21, 9, 0, 1, tzinfo=timezone.utc)
        tid = j.log_session_plan(
            {
                "symbol": "SPX",
                "briefing": {"direction": "neutral"},
                "intent": {"allow_entry": True},
                "entry_plan": {"entry": "proposed"},
            }
        )
        j.log_entry_execution(trace_id=tid, symbol="SPX", entry_timestamp=base_ts)
        j.log_close_decision(trace_id=tid, close_decision={"symbol": "SPX"}, timestamp=base_ts)
        j.log_close_execution(trace_id=tid, symbol="SPX", close_timestamp=base_ts, pnl_realized=10.0)
        raw = p.read_text(encoding="utf-8")
        for line in raw.splitlines():
            if line.strip():
                json.loads(line)

    def test_entry_close_cycle_reconstructible_with_canonical_fields(self, tmp_path: Path):
        p = tmp_path / "cycle.jsonl"
        j = OptionsPaperJournal(path=p)
        base_ts = datetime(2026, 4, 21, 9, 0, tzinfo=timezone.utc)
        tid = j.log_session_plan(
            {
                "symbol": "SPX",
                "briefing": {"direction": "neutral", "dte": 14},
                "intent": {"allow_entry": True},
                "entry_plan": {"entry": "proposed", "recommended_strategy": "iron_condor"},
            }
        )
        j.log_entry_execution(
            trace_id=tid,
            symbol="SPX",
            entry_timestamp=base_ts,
            executed_entry={
                "structure": "iron_condor",
                "credit": 2.2,
                "entry_mid": 2.15,
                "dte": 14,
                "legs": [
                    {"kind": "put", "side": "short", "strike": 5000, "expiration": "2026-05-15", "qty": 1},
                    {"kind": "put", "side": "long", "strike": 4980, "expiration": "2026-05-15", "qty": 1},
                    {"kind": "call", "side": "short", "strike": 5300, "expiration": "2026-05-15", "qty": 1},
                    {"kind": "call", "side": "long", "strike": 5320, "expiration": "2026-05-15", "qty": 1},
                ],
            },
            source="manual",
        )
        j.log_close_execution(
            trace_id=tid,
            symbol="SPX",
            close_timestamp=base_ts,
            executed_close={"debit": 1.0, "close_mid": 1.0},
            pnl_realized=120.0,
            close_reason="tp_50pct",
            source="autoclose",
            autoclose_applied=True,
        )
        rows = _read_jsonl(p)
        by_tid = [r for r in rows if r.get("trace_id") == tid]
        assert len(by_tid) == 3
        entry = next(r for r in by_tid if r["event_type"] == "entry_execution")
        close = next(r for r in by_tid if r["event_type"] == "close_execution")
        assert entry["underlying"] == "SPX"
        assert entry["structure_type"] == "iron_condor"
        assert entry["dte_at_entry"] == 14
        assert isinstance(entry["legs"], list) and len(entry["legs"]) == 4
        assert entry["entry_credit"] == pytest.approx(2.2)
        assert close["close_reason"] == "tp_50pct"
        assert close["autoclose_applied"] is True
        assert close["pnl_usd"] == pytest.approx(120.0)
        assert close["status"] == "closed"

    def test_orchestrator_optional_journal_integration(self, tmp_path: Path):
        p = tmp_path / "orch.jsonl"
        journal = OptionsPaperJournal(path=p)
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
            capital=25_000.0,
        )
        assert "trace_id" in out
        assert isinstance(out["trace_id"], str) and out["trace_id"]
        rows = _read_jsonl(p)
        assert len(rows) == 1
        assert rows[0]["event_type"] == "session_plan"
        assert rows[0]["trace_id"] == out["trace_id"]

    def test_orchestrator_includes_market_open_runtime(self, tmp_path: Path):
        p = tmp_path / "mor.jsonl"
        journal = OptionsPaperJournal(path=p)
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
            capital=25_000.0,
        )
        mor = out.get("market_open_runtime") or {}
        assert isinstance(mor, dict)
        assert "config_path" in mor
        assert "config_loaded" in mor
        assert "max_open_positions" in mor
        assert "kelly_fraction" in mor
        assert "schedule_et" in mor

    def test_orchestrator_visual_signal_breach_in_briefing(self, tmp_path: Path):
        p = tmp_path / "vs.jsonl"
        journal = OptionsPaperJournal(path=p)
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok(spot=5200.0)
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner(), journal=journal)
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            capital=25_000.0,
            spot=4980.0,
            support_levels=[5000.0],
            resistance_levels=[5300.0],
        )
        vs = out["briefing"].get("visual_signal") or {}
        assert vs.get("availability") == "ok"
        assert vs.get("breach_detected") is True
        assert vs.get("source") == "visual_signal_adapter"
        assert "visual_signal_adapter_breach" in (out["briefing"].get("quality_flags") or [])
        assert bool(out["briefing"].get("breach_context", {}).get("breach_detected")) is True

    def test_orchestrator_visual_signal_adapter_exception_degrades(self, tmp_path: Path, monkeypatch):
        p = tmp_path / "vse.jsonl"
        journal = OptionsPaperJournal(path=p)
        iv = MagicMock()
        iv.get_iv_rank.return_value = _iv_ok(spot=5200.0)
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")

        class _BoomAdapter:
            def __init__(self, **kwargs: object) -> None:
                pass

            def analyze_price_context(self, **kwargs: object) -> dict:
                raise RuntimeError("simulated_adapter_failure")

        monkeypatch.setattr(orch_mod, "VisualSignalAdapter", _BoomAdapter)
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner(), journal=journal)
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            capital=25_000.0,
            spot=5150.0,
            support_levels=[5000.0],
        )
        vs = out["briefing"].get("visual_signal") or {}
        assert vs.get("availability") == "error"
        assert "adapter_exception" in " ".join(vs.get("notes") or [])

    def test_orchestrator_visual_signal_unavailable_without_spot(self, tmp_path: Path):
        p = tmp_path / "vss.jsonl"
        journal = OptionsPaperJournal(path=p)
        iv = MagicMock()
        iv.get_iv_rank.return_value = {
            "symbol": "SPX",
            "iv_current": 0.18,
            "iv_rank": 40.0,
            "iv_hv_ratio": 1.05,
            "quality": "ok",
            "method": "test_no_spot",
        }
        engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
        orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner(), journal=journal)
        out = orch.build_session_plan(
            symbol="SPX",
            direction="neutral",
            regime="ranging",
            gamma_regime="long_gamma",
            dte_mode="8to21",
            capital=25_000.0,
            spot=None,
        )
        assert out["briefing"].get("visual_signal", {}).get("availability") == "unavailable"
