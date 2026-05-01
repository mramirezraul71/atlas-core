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
    assert ns.mode == "paper"
    assert ns.source == "manual"


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
    assert rows[0]["entry_credit"] == pytest.approx(2.5)
    assert rows[0]["mode"] == "paper"
    assert rows[0]["source"] == "manual"
    assert rows[0]["journal_version"] == "1.0"
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
            "--reason",
            "manual_take_profit",
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
    assert len(rows) == 2
    assert rows[0]["event_type"] == "close_decision"
    assert rows[0]["trace_id"] == "trace-close-1"
    assert rows[0]["symbol"] == "SPX"
    cd = rows[0]["payload"]["close_decision"]
    assert cd["decision"] == "execute_close"
    assert cd["source"] == "options_trade_log_cli.close"
    assert cd["reason"] == "manual_take_profit"
    assert cd["close_type"] == "full"
    assert cd["cli_context"]["pnl_realized_declared"] == pytest.approx(165.5)
    assert "T" in rows[0]["timestamp"] or "Z" in rows[0]["timestamp"]
    assert rows[1]["event_type"] == "close_execution"
    assert rows[1]["trace_id"] == "trace-close-1"
    assert rows[1]["symbol"] == "SPX"
    assert rows[1]["payload"]["pnl_realized"] == pytest.approx(165.5)
    assert rows[1]["payload"]["executed_close"]["debit"] == pytest.approx(0.45)
    assert rows[1]["close_type"] == "full"
    assert rows[1]["close_reason"] == "manual_take_profit"
    assert rows[1]["pnl_usd"] == pytest.approx(165.5)
    assert rows[1]["mode"] == "paper"


def test_run_close_rejects_blank_trace_id(tmp_path: Path, capsys):
    jpath = tmp_path / "bad.jsonl"
    code = cli.run(
        [
            "close",
            "--journal-path",
            str(jpath),
            "--trace-id",
            "   ",
            "--symbol",
            "SPX",
        ]
    )
    assert code == 2
    err = capsys.readouterr().err
    assert "trace_id" in err.lower() or "vacío" in err
    assert _read_jsonl(jpath) == []


def test_run_close_decision_minimal_no_invented_reason(tmp_path: Path, capsys):
    """Solo trace/symbol: decisión explícita execute_close sin clave reason inventada."""
    jpath = tmp_path / "min.jsonl"
    code = cli.run(
        [
            "close",
            "--journal-path",
            str(jpath),
            "--trace-id",
            "t-min",
            "--symbol",
            "qqq",
        ]
    )
    assert code == 0
    rows = _read_jsonl(jpath)
    assert len(rows) == 2
    cd = rows[0]["payload"]["close_decision"]
    assert cd["symbol"] == "QQQ"
    assert "reason" not in cd
    assert "cli_context" not in cd
    assert rows[0]["notes"] == ["paper_close_cli"]
    assert rows[1]["close_reason"] is None
