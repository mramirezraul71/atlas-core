from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from atlas_code_quant.options.paper_journal_stats import OptionsPaperJournalStats


def _write_jsonl(path: Path, lines: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def test_stats_empty_journal(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(p, [])
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    g = stats.to_dict_global()
    assert g["closed_trades_total"] == 0
    assert g["win_rate_pct"] == 0.0
    assert g["profit_factor"] == 0.0
    assert g["equity_usd"] == 10_000.0
    assert g["drawdown_pct"] == 0.0


def test_stats_only_wins_profit_factor_capped(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        p,
        [
            '{"event_type":"close_execution","trace_id":"a","timestamp_utc":"2026-04-20T10:00:00Z","symbol":"SPY","close_reason":"tp","pnl_usd":12.5}',
            '{"event_type":"close_execution","trace_id":"b","timestamp_utc":"2026-04-20T10:30:00Z","symbol":"SPY","close_reason":"tp","pnl_usd":7.5}',
        ],
    )
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    g = stats.to_dict_global()
    assert g["closed_trades_total"] == 2
    assert g["wins_total"] == 2
    assert g["losses_total"] == 0
    assert g["win_rate_pct"] == 100.0
    assert g["profit_factor"] == 99.0
    assert g["pnl_total_usd"] == 20.0


def test_stats_mixed_wins_losses_and_drawdown(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        p,
        [
            '{"event_type":"close_execution","trace_id":"a","timestamp_utc":"2026-04-20T10:00:00Z","symbol":"SPY","close_reason":"tp","pnl_usd":100.0}',
            '{"event_type":"close_execution","trace_id":"b","timestamp_utc":"2026-04-20T10:10:00Z","symbol":"SPY","close_reason":"sl","pnl_usd":-40.0}',
            '{"event_type":"close_execution","trace_id":"c","timestamp_utc":"2026-04-20T10:20:00Z","symbol":"SPY","close_reason":"tp","pnl_usd":60.0}',
        ],
    )
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    g = stats.to_dict_global()
    assert g["closed_trades_total"] == 3
    assert round(g["win_rate_pct"], 4) == round(2 / 3 * 100, 4)
    assert round(g["profit_factor"], 4) == round(160.0 / 40.0, 4)
    assert g["equity_usd"] == 10_120.0
    assert g["max_drawdown_usd"] == 40.0


def test_stats_rolling_7d_and_strategy_regime(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        p,
        [
            '{"event_type":"session_plan","trace_id":"t_old","timestamp_utc":"2026-04-10T10:00:00Z","symbol":"IWM","payload":{"session_plan":{"briefing":{"gamma_regime":"short_gamma","dte_mode":"8to21"},"entry_plan":{"recommended_strategy":"iron_condor"}}}}',
            '{"event_type":"close_execution","trace_id":"t_old","timestamp_utc":"2026-04-10T10:05:00Z","symbol":"IWM","close_reason":"stop_loss","pnl_usd":-10.0}',
            '{"event_type":"session_plan","trace_id":"t_new","timestamp_utc":"2026-04-19T10:00:00Z","symbol":"IWM","payload":{"session_plan":{"briefing":{"gamma_regime":"long_gamma","dte_mode":"8to21"},"entry_plan":{"recommended_strategy":"bull_call_debit_spread"}}}}',
            '{"event_type":"close_execution","trace_id":"t_new","timestamp_utc":"2026-04-19T10:05:00Z","symbol":"IWM","close_reason":"take_profit","pnl_usd":20.0}',
        ],
    )
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    rolling = stats.rolling_7d
    assert rolling["closed_trades"] == 1
    assert rolling["win_rate_pct"] == 100.0
    by_regime = stats.to_dict_by_strategy_regime()
    assert by_regime["iron_condor"]["short_gamma"] == 0.0
    assert by_regime["bull_call_debit_spread"]["long_gamma"] == 100.0


def test_stats_prefers_direct_close_execution_fields(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        p,
        [
            '{"event_type":"close_execution","trace_id":"t1","timestamp_utc":"2026-04-20T10:05:00Z","symbol":"IWM","strategy_type":"iron_condor","gamma_regime":"short_gamma","dte_mode":"8to21","close_reason":"take_profit","pnl_usd":20.0}',
            '{"event_type":"close_execution","trace_id":"t2","timestamp_utc":"2026-04-20T10:07:00Z","symbol":"IWM","strategy_type":"iron_condor","gamma_regime":"short_gamma","dte_mode":"8to21","close_reason":"stop_loss","pnl_usd":-10.0}',
        ],
    )
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    by_regime = stats.to_dict_by_strategy_regime()
    assert by_regime["iron_condor"]["short_gamma"] == 50.0


def test_stats_excludes_non_executable_closes_and_counts_blocked_sessions(tmp_path: Path):
    p = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        p,
        [
            '{"event_type":"session_plan","trace_id":"t1","timestamp_utc":"2026-04-20T09:00:00Z","payload":{"session_plan":{"entry_allowed":true,"intent":{"force_no_trade":false}}}}',
            '{"event_type":"entry_blocked","trace_id":"t1","timestamp_utc":"2026-04-20T09:00:01Z","entry_executable":false,"position_size_units":0,"payload":{"blocked_reason":"min_contract_not_reached"}}',
            '{"event_type":"close_execution","trace_id":"t1","timestamp_utc":"2026-04-20T09:00:02Z","entry_executable":false,"position_size_units":0,"pnl_usd":999.0}',
            '{"event_type":"close_execution","trace_id":"t2","timestamp_utc":"2026-04-20T09:05:00Z","entry_executable":true,"position_size_units":1,"strategy_type":"iron_condor","gamma_regime":"long_gamma","pnl_usd":10.0}',
        ],
    )
    stats = OptionsPaperJournalStats.load_from_file(
        p,
        initial_capital=10_000.0,
        now_utc=datetime(2026, 4, 20, 16, 0, tzinfo=timezone.utc),
    )
    g = stats.to_dict_global()
    assert g["closed_trades_total"] == 1
    assert g["blocked_sessions_today"] == 1
    assert g["go_sessions_today"] == 1
    api = stats.to_api_dict()
    assert api["summary"]["closed_trades_total"] == 1
