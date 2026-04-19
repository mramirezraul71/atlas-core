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
            notes=["paper_close"],
        )
        rows = _read_jsonl(p)
        assert len(rows) == 2
        assert rows[0]["event_type"] == "close_decision"
        assert rows[0]["payload"]["close_decision"]["action"] == "close_winners"
        assert rows[1]["event_type"] == "close_execution"
        assert rows[1]["payload"]["pnl_realized"] == pytest.approx(120.5)

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
