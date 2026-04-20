from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_code_quant.api.routes import options as options_module
from atlas_code_quant.api.routes.options import router as options_router


def _build_client() -> TestClient:
    app = FastAPI()
    app.include_router(options_router)
    return TestClient(app)


def _write_jsonl(path: Path, rows: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")


def _reset_cache() -> None:
    options_module._paper_perf_cache_payload = None
    options_module._paper_perf_cache_expires_at = 0.0


def test_paper_performance_endpoint_empty_journal(tmp_path: Path, monkeypatch):
    _reset_cache()
    journal = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(journal, [])
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_JOURNAL_PATH", str(journal))
    client = _build_client()
    resp = client.get("/options/paper-performance")
    assert resp.status_code == 200
    payload = resp.json()
    assert payload["ok"] is True
    assert payload["summary"]["closed_trades_total"] == 0
    assert payload["summary"]["win_rate_pct"] == 0.0
    assert payload["summary"]["profit_factor"] == 0.0
    assert payload["summary"]["equity_usd"] == 10_000.0
    assert payload["message"] == "no trades yet"


def test_paper_performance_endpoint_mixed_wins_losses(tmp_path: Path, monkeypatch):
    _reset_cache()
    journal = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(
        journal,
        [
            '{"event_type":"session_plan","trace_id":"t1","timestamp_utc":"2026-04-20T12:00:00Z","symbol":"IWM","payload":{"session_plan":{"briefing":{"gamma_regime":"long_gamma","dte_mode":"8to21"},"entry_plan":{"recommended_strategy":"bull_call_debit_spread"}}}}',
            '{"event_type":"close_execution","trace_id":"t1","timestamp_utc":"2026-04-20T12:01:00Z","symbol":"IWM","close_reason":"take_profit","pnl_usd":10.0}',
            '{"event_type":"session_plan","trace_id":"t2","timestamp_utc":"2026-04-20T12:02:00Z","symbol":"IWM","payload":{"session_plan":{"briefing":{"gamma_regime":"short_gamma","dte_mode":"8to21"},"entry_plan":{"recommended_strategy":"iron_condor"}}}}',
            '{"event_type":"close_execution","trace_id":"t2","timestamp_utc":"2026-04-20T12:03:00Z","symbol":"IWM","close_reason":"stop_loss","pnl_usd":-4.0}',
        ],
    )
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_JOURNAL_PATH", str(journal))
    client = _build_client()
    resp = client.get("/options/paper-performance")
    assert resp.status_code == 200
    payload = resp.json()
    assert payload["ok"] is True
    summary = payload["summary"]
    assert summary["closed_trades_total"] == 2
    assert summary["wins_total"] == 1
    assert summary["losses_total"] == 1
    assert summary["win_rate_pct"] == 50.0
    assert summary["profit_factor"] == 2.5
    assert summary["pnl_total_usd"] == 6.0
    assert payload["by_strategy_regime"]["bull_call_debit_spread"]["long_gamma"] == 100.0
    assert payload["by_strategy_regime"]["iron_condor"]["short_gamma"] == 0.0


def test_paper_performance_cache_hit_within_ttl(tmp_path: Path, monkeypatch):
    _reset_cache()
    journal = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(journal, [])
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_JOURNAL_PATH", str(journal))
    monkeypatch.setenv("QUANT_OPTIONS_PERF_CACHE_TTL_SEC", "10")

    counter = {"calls": 0}
    real_load = options_module.OptionsPaperJournalStats.load_from_file

    def _wrapped_loader(path):
        counter["calls"] += 1
        return real_load(path)

    monkeypatch.setattr(options_module.OptionsPaperJournalStats, "load_from_file", _wrapped_loader)
    client = _build_client()
    first = client.get("/options/paper-performance").json()
    second = client.get("/options/paper-performance").json()

    assert counter["calls"] == 1
    assert first["cache_hit"] is False
    assert second["cache_hit"] is True
    assert second["cache_ttl_seconds"] == 10.0


def test_paper_performance_cache_expires_and_refreshes(tmp_path: Path, monkeypatch):
    _reset_cache()
    journal = tmp_path / "options_paper_journal.jsonl"
    _write_jsonl(journal, [])
    monkeypatch.setenv("QUANT_OPTIONS_RUNTIME_JOURNAL_PATH", str(journal))
    monkeypatch.setenv("QUANT_OPTIONS_PERF_CACHE_TTL_SEC", "10")

    fake_time = SimpleNamespace(now=1000.0)

    def _fake_time():
        return fake_time.now

    monkeypatch.setattr(options_module.time, "time", _fake_time)

    counter = {"calls": 0}
    real_load = options_module.OptionsPaperJournalStats.load_from_file

    def _wrapped_loader(path):
        counter["calls"] += 1
        return real_load(path)

    monkeypatch.setattr(options_module.OptionsPaperJournalStats, "load_from_file", _wrapped_loader)
    client = _build_client()
    client.get("/options/paper-performance")
    fake_time.now = 1005.0
    client.get("/options/paper-performance")
    fake_time.now = 1012.0
    third = client.get("/options/paper-performance").json()

    assert counter["calls"] == 2
    assert third["cache_hit"] is False
