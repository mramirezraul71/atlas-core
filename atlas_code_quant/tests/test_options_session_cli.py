"""Tests mínimos del CLI de plan de sesión opciones (paper-only)."""
from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

from atlas_code_quant.options.session_briefing import SessionBriefingEngine
from atlas_code_quant.scripts import options_session_cli as cli


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


def _fake_briefing_engine() -> SessionBriefingEngine:
    iv = MagicMock()
    iv.get_iv_rank.return_value = _iv_ok()
    return SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")


def test_parse_args_minimal():
    ns = cli.parse_args(["--symbol", "spx", "--capital", "10000"])
    assert ns.symbol == "spx"
    assert ns.capital == 10000.0
    assert ns.direction is None
    assert ns.journal_path is None


def test_run_returns_zero_and_prints_header_and_decision(capsys, tmp_path: Path, monkeypatch):
    monkeypatch.setattr(cli, "_default_briefing_engine", _fake_briefing_engine)
    jpath = tmp_path / "cli_journal.jsonl"
    code = cli.run(
        [
            "--symbol",
            "SPX",
            "--capital",
            "10000",
            "--direction",
            "neutral",
            "--regime",
            "ranging",
            "--gamma-regime",
            "long_gamma",
            "--dte-mode",
            "8to21",
            "--journal-path",
            str(jpath),
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "OPTIONS SESSION PLAN — SPX" in out
    assert "DECISIÓN PAPER:" in out
    assert "trace_id" in out
    assert jpath.is_file()
