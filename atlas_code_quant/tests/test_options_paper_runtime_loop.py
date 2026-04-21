from __future__ import annotations

import asyncio
import json
from pathlib import Path

from atlas_code_quant.options.paper_runtime_loop import (
    OptionsPaperRuntimeLoop,
    options_runtime_loop_enabled,
)


class _FakeOrchestrator:
    def build_session_plan(self, *, symbol: str, capital: float) -> dict:
        return {
            "automation_mode": "paper_only",
            "symbol": symbol,
            "entry_allowed": True,
            "briefing": {
                "dte_mode": "8to21",
                "dte": 21,
                "iv_rank": 42.0,
            },
            "intent": {"allow_entry": True, "force_no_trade": False},
            "entry_plan": {
                "entry": "proposed",
                "recommended_strategy": "iron_condor",
                "max_risk_budget_dollars": 120.0,
                "position_size_units": 1,
            },
            "pipeline_quality_flags": [],
        }


class _BlockedOrchestrator:
    def build_session_plan(self, *, symbol: str, capital: float) -> dict:
        return {
            "automation_mode": "paper_only",
            "symbol": symbol,
            "entry_allowed": True,
            "briefing": {
                "gamma_regime": "unknown",
                "dte_mode": "8to21",
            },
            "intent": {"allow_entry": True, "force_no_trade": False},
            "entry_plan": {
                "entry": "blocked",
                "recommended_strategy": "iron_condor",
                "position_size_units": None,
                "size_blocked_reason": "min_contract_not_reached",
            },
            "pipeline_quality_flags": [],
        }


def _read_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


def test_runtime_ticks_generate_entry_and_autoclose(tmp_path: Path):
    jpath = tmp_path / "options_paper_journal.jsonl"
    runtime = OptionsPaperRuntimeLoop(
        symbol="IWM",
        capital=10_000.0,
        session_interval_sec=60.0,
        autoclose_interval_sec=15.0,
        close_after_sec=45.0,
        journal_path=jpath,
        orchestrator=_FakeOrchestrator(),
    )

    asyncio.run(runtime._run_session_tick())
    assert jpath.is_file()
    rows_after_session = _read_jsonl(jpath)
    event_types = [row.get("event_type") for row in rows_after_session]
    assert "session_plan" in event_types
    assert "entry_execution" in event_types
    assert len(runtime._open_positions) == 1

    # Fuerza madurez de la posición para disparar TP/SL en el siguiente tick.
    only_pos = next(iter(runtime._open_positions.values()))
    only_pos.opened_at_ts -= 60.0
    asyncio.run(runtime._run_autoclose_tick())

    rows_after_close = _read_jsonl(jpath)
    by_type = [row.get("event_type") for row in rows_after_close]
    assert "close_decision" in by_type
    assert "close_execution" in by_type

    entry_tid = next(row.get("trace_id") for row in rows_after_close if row.get("event_type") == "entry_execution")
    close_tid = next(row.get("trace_id") for row in rows_after_close if row.get("event_type") == "close_execution")
    assert entry_tid == close_tid
    assert len(runtime._open_positions) == 0


def test_options_runtime_enabled_flag(monkeypatch):
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_LOOP_ENABLED", "false")
    assert options_runtime_loop_enabled() is False
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_LOOP_ENABLED", "true")
    assert options_runtime_loop_enabled() is True


def test_runtime_writes_entry_blocked_when_size_not_executable(tmp_path: Path):
    jpath = tmp_path / "options_paper_journal.jsonl"
    runtime = OptionsPaperRuntimeLoop(
        symbol="IWM",
        capital=10_000.0,
        session_interval_sec=60.0,
        autoclose_interval_sec=15.0,
        close_after_sec=45.0,
        journal_path=jpath,
        orchestrator=_BlockedOrchestrator(),
    )
    asyncio.run(runtime._run_session_tick())
    rows = _read_jsonl(jpath)
    events = [r.get("event_type") for r in rows]
    assert "session_plan" in events
    assert "entry_blocked" in events
    assert "entry_execution" not in events
    blocked = next(r for r in rows if r.get("event_type") == "entry_blocked")
    assert blocked.get("entry_executable") is False
    assert blocked.get("position_size_units") == 0
