"""Tests del CLI de registro paper (entry/close) en OptionsPaperJournal."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from atlas_code_quant.scripts import options_trade_log_cli as cli


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


def test_parse_args_entry():
    ns = cli.parse_args(
        [
            "entry",
            "--trace-id",
            "tid-1",
            "--symbol",
            "SPX",
            "--structure",
            "iron_condor",
            "--credit",
            "2.1",
            "--dte",
            "14",
        ]
    )
    assert ns.command == "entry"
    assert ns.trace_id == "tid-1"
    assert ns.symbol == "SPX"
    assert ns.structure == "iron_condor"
    assert ns.credit == pytest.approx(2.1)
    assert ns.dte == 14
    assert ns.journal_path is None


def test_run_entry_writes_jsonl(tmp_path: Path, capsys):
    jpath = tmp_path / "t.jsonl"
    code = cli.run(
        [
            "entry",
            "--journal-path",
            str(jpath),
            "--trace-id",
            "trace-entry-1",
            "--symbol",
            "spx",
            "--structure",
            "iron_condor",
            "--credit",
            "2.5",
            "--dte",
            "21",
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "ENTRY LOGGED" in out
    assert "trace-entry-1" in out
    assert "SPX" in out
    rows = _read_jsonl(jpath)
    assert len(rows) == 1
    assert rows[0]["event_type"] == "entry_execution"
    assert rows[0]["trace_id"] == "trace-entry-1"
    assert rows[0]["symbol"] == "SPX"
    assert rows[0]["payload"]["executed_entry"]["structure"] == "iron_condor"
    assert rows[0]["payload"]["executed_entry"]["credit"] == pytest.approx(2.5)
    assert rows[0]["notes"] == ["paper_entry_cli"]


def test_run_close_writes_jsonl_pnl(tmp_path: Path, capsys):
    jpath = tmp_path / "c.jsonl"
    code = cli.run(
        [
            "close",
            "--journal-path",
            str(jpath),
            "--trace-id",
            "trace-close-1",
            "--symbol",
            "SPX",
            "--debit",
            "0.45",
            "--exit-mid",
            "0.44",
            "--pnl",
            "165.5",
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "CLOSE LOGGED" in out
    assert "165.50" in out or "165.5" in out
    rows = _read_jsonl(jpath)
    assert len(rows) == 1
    assert rows[0]["event_type"] == "close_execution"
    assert rows[0]["trace_id"] == "trace-close-1"
    assert rows[0]["symbol"] == "SPX"
    assert rows[0]["payload"]["pnl_realized"] == pytest.approx(165.5)
    assert rows[0]["payload"]["executed_close"]["debit"] == pytest.approx(0.45)
