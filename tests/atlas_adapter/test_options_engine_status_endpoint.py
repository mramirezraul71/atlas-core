from __future__ import annotations

import json
import urllib.error
import urllib.parse

from fastapi.testclient import TestClient

from atlas_adapter import atlas_http_api


def test_options_engine_status_endpoint_works_without_backtesting(monkeypatch):
    samples = {
        "atlas_options_session_go_nogo": 1.0,
        "atlas_options_paper_trades_closed_today": 0.0,
        "atlas_options_paper_trades_open": 0.0,
        "atlas_options_journal_events_today": 1.0,
        "atlas_options_journal_sessions_today": 1.0,
        "atlas_options_journal_last_write_age_seconds": 13.0,
        "atlas_options_iv_rank_quality_score": 1.0,
        "atlas_options_paper_win_rate_ratio": None,
        "atlas_options_paper_profit_factor": None,
        "atlas_options_iv_rank_value": None,
        "atlas_options_iv_rank_current": None,
    }

    def fake_urlopen(url: str, timeout: int = 5):
        class _Resp:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def read(self):
                import json
                import urllib.parse

                query = urllib.parse.parse_qs(urllib.parse.urlparse(url).query).get("query", [""])[0]
                value = samples.get(query)
                if value is None:
                    payload = {"status": "success", "data": {"result": []}}
                else:
                    payload = {
                        "status": "success",
                        "data": {
                            "result": [
                                {
                                    "metric": {"__name__": query},
                                    "value": [1776690000.0, str(value)],
                                }
                            ]
                        },
                    }
                return json.dumps(payload).encode("utf-8")

        return _Resp()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)

    client = TestClient(atlas_http_api.app)
    res = client.get("/api/options-engine-status")

    assert res.status_code == 200
    body = res.json()
    assert body["ok"] is True
    assert body["status"] == "GO"
    assert body["automation_mode"] == "paper_only"
    assert body["loop_mode"] == "session_only"
    assert body["trades_completed_total"] == 0
    assert body["trades_closed_today"] == 0
    assert body["iv_rank_current"] is None
    assert body["iv_rank_quality_tier"] == 2


def test_options_engine_status_uses_paper_performance_endpoint(monkeypatch):
    prom_samples = {
        "atlas_options_session_go_nogo": 1.0,
        "atlas_options_paper_trades_closed_today": 5.0,
        "atlas_options_paper_trades_open": 1.0,
        "atlas_options_journal_events_today": 8.0,
        "atlas_options_journal_sessions_today": 3.0,
        "atlas_options_journal_last_write_age_seconds": 12.0,
        "atlas_options_iv_rank_quality_score": 1.0,
        "atlas_options_iv_rank_value": 44.2,
        "atlas_options_paper_win_rate_ratio": 0.5,
        "atlas_options_paper_profit_factor": 1.1,
    }

    perf_payload = {
        "ok": True,
        "summary": {
            "closed_trades_total": 39,
            "win_rate_pct": 78.3,
            "profit_factor": 1.87,
        },
    }

    def fake_urlopen(url: str, timeout: int = 5):
        class _Resp:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def read(self):
                if url.endswith("/options/paper-performance"):
                    return json.dumps(perf_payload).encode("utf-8")
                query = urllib.parse.parse_qs(urllib.parse.urlparse(url).query).get("query", [""])[0]
                value = prom_samples.get(query)
                if value is None:
                    payload = {"status": "success", "data": {"result": []}}
                else:
                    payload = {
                        "status": "success",
                        "data": {"result": [{"metric": {"__name__": query}, "value": [1776690000.0, str(value)]}]},
                    }
                return json.dumps(payload).encode("utf-8")

        return _Resp()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    client = TestClient(atlas_http_api.app)
    body = client.get("/api/options-engine-status").json()
    assert body["ok"] is True
    assert body["trades_completed_total"] == 39
    assert body["wr_basic"] == 78.3
    assert body["pf_basic"] == 1.87
    assert "fallback" not in str(body.get("notes") or "").lower()


def test_options_engine_status_fallback_when_paper_performance_fails(monkeypatch):
    prom_samples = {
        "atlas_options_session_go_nogo": 1.0,
        "atlas_options_paper_trades_closed_today": 7.0,
        "atlas_options_paper_trades_open": 0.0,
        "atlas_options_journal_events_today": 9.0,
        "atlas_options_journal_sessions_today": 4.0,
        "atlas_options_journal_last_write_age_seconds": 11.0,
        "atlas_options_iv_rank_quality_score": 1.0,
        "atlas_options_iv_rank_value": 45.0,
        "atlas_options_paper_win_rate_ratio": 0.66,
        "atlas_options_paper_profit_factor": 1.42,
    }

    def fake_urlopen(url: str, timeout: int = 5):
        class _Resp:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def read(self):
                query = urllib.parse.parse_qs(urllib.parse.urlparse(url).query).get("query", [""])[0]
                value = prom_samples.get(query)
                if value is None:
                    payload = {"status": "success", "data": {"result": []}}
                else:
                    payload = {
                        "status": "success",
                        "data": {"result": [{"metric": {"__name__": query}, "value": [1776690000.0, str(value)]}]},
                    }
                return json.dumps(payload).encode("utf-8")

        if url.endswith("/options/paper-performance"):
            raise urllib.error.URLError("connection refused")
        return _Resp()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    client = TestClient(atlas_http_api.app)
    body = client.get("/api/options-engine-status").json()
    assert body["ok"] is True
    assert body["trades_completed_total"] == 7
    assert body["wr_basic"] == 0.66
    assert body["pf_basic"] == 1.42
    assert "paper performance metrics unavailable; using fallback metrics" in (body.get("notes") or "")


def test_options_engine_status_degrades_when_go_sessions_are_blocked(monkeypatch):
    prom_samples = {
        "atlas_options_session_go_nogo": 1.0,
        "atlas_options_paper_trades_closed_today": 0.0,
        "atlas_options_paper_trades_open": 0.0,
        "atlas_options_journal_events_today": 9.0,
        "atlas_options_journal_sessions_today": 4.0,
        "atlas_options_journal_last_write_age_seconds": 11.0,
        "atlas_options_iv_rank_quality_score": 1.0,
        "atlas_options_iv_rank_value": 45.0,
        "atlas_options_paper_win_rate_ratio": 0.0,
        "atlas_options_paper_profit_factor": 0.0,
    }
    perf_payload = {
        "ok": True,
        "summary": {
            "closed_trades_total": 0,
            "win_rate_pct": 0.0,
            "profit_factor": 0.0,
            "go_sessions_today": 12,
            "blocked_sessions_today": 3,
        },
    }

    def fake_urlopen(url: str, timeout: int = 5):
        class _Resp:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

            def read(self):
                if url.endswith("/options/paper-performance"):
                    return json.dumps(perf_payload).encode("utf-8")
                query = urllib.parse.parse_qs(urllib.parse.urlparse(url).query).get("query", [""])[0]
                value = prom_samples.get(query)
                if value is None:
                    payload = {"status": "success", "data": {"result": []}}
                else:
                    payload = {
                        "status": "success",
                        "data": {"result": [{"metric": {"__name__": query}, "value": [1776690000.0, str(value)]}]},
                    }
                return json.dumps(payload).encode("utf-8")

        return _Resp()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    client = TestClient(atlas_http_api.app)
    body = client.get("/api/options-engine-status").json()
    assert body["ok"] is True
    assert body["status"] == "DEGRADED"
    assert body["go_sessions_today"] == 12
    assert body["blocked_sessions_today"] == 3
