from __future__ import annotations

import asyncio
import json
from pathlib import Path

from atlas_code_quant.options.paper_runtime_loop import OptionsPaperRuntimeLoop


class _IdempotentOrchestrator:
    def __init__(self) -> None:
        self.calls = 0

    def build_session_plan(self, *, symbol: str, capital: float) -> dict:
        self.calls += 1
        return {
            "automation_mode": "paper_only",
            "symbol": symbol,
            "session_id": "session-ny-open",
            "entry_allowed": True,
            "briefing": {
                "gamma_regime": "long_gamma",
                "dte_mode": "8to21",
                "dte": 21,
            },
            "intent": {"allow_entry": True, "force_no_trade": False},
            "entry_plan": {
                "entry": "proposed",
                "recommended_strategy": "iron_condor",
                "max_risk_budget_dollars": 120.0,
                "position_size_units": 1,
                "entry_owner": "autoclose_engine",
                "exit_rules": {"take_profit_pct": 0.5},
                "safety_flags": ["paper_only"],
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


def test_runtime_loop_is_idempotent_by_session_id(tmp_path: Path):
    journal = tmp_path / "options_paper_journal.jsonl"
    runtime = OptionsPaperRuntimeLoop(
        symbol="IWM",
        capital=10_000.0,
        session_interval_sec=60.0,
        autoclose_interval_sec=15.0,
        close_after_sec=45.0,
        journal_path=journal,
        orchestrator=_IdempotentOrchestrator(),
    )

    asyncio.run(runtime._run_session_tick())
    asyncio.run(runtime._run_session_tick())

    rows = _read_jsonl(journal)
    entry_rows = [row for row in rows if row.get("event_type") == "entry_execution"]
    session_rows = [row for row in rows if row.get("event_type") == "session_plan"]

    assert len(session_rows) == 2
    assert len(entry_rows) == 1
    assert len(runtime._open_positions) == 1
    only_position = next(iter(runtime._open_positions.values()))
    assert only_position.session_key == "session-ny-open"
    assert only_position.owner == "autoclose_engine"
    assert only_position.exit_rules == {"take_profit_pct": 0.5}
    assert only_position.safety_flags == ["paper_only"]


def test_runtime_loop_skips_new_entry_when_owner_position_is_open(tmp_path: Path):
    journal = tmp_path / "options_paper_journal.jsonl"
    orchestrator = _IdempotentOrchestrator()
    runtime = OptionsPaperRuntimeLoop(
        symbol="IWM",
        capital=10_000.0,
        session_interval_sec=60.0,
        autoclose_interval_sec=15.0,
        close_after_sec=45.0,
        journal_path=journal,
        orchestrator=orchestrator,
    )

    asyncio.run(runtime._run_session_tick())
    original_builder = orchestrator.build_session_plan

    def _new_session(*, symbol: str, capital: float) -> dict:
        payload = original_builder(symbol=symbol, capital=capital)
        payload["session_id"] = "session-second-pass"
        return payload

    orchestrator.build_session_plan = _new_session  # type: ignore[method-assign]
    asyncio.run(runtime._run_session_tick())

    rows = _read_jsonl(journal)
    assert len([row for row in rows if row.get("event_type") == "entry_execution"]) == 1
    assert len(runtime._open_positions) == 1
