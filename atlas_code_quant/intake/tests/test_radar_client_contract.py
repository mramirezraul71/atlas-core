from __future__ import annotations

from urllib.error import HTTPError

import pytest

from atlas_code_quant.intake.radar_client import RadarOpportunityClient


def test_radar_client_parses_opportunities_response(monkeypatch: pytest.MonkeyPatch) -> None:
    client = RadarOpportunityClient(
        opportunities_url="http://radar.local/api/radar/opportunities",
        enabled=True,
        timeout_sec=1.0,
        default_limit=5,
        default_min_score=0.0,
    )

    def _fake_request_json(self, url: str):
        assert "limit=5" in url
        return (
            200,
            {
                "ok": True,
                "trace_id": "trace-f3-1",
                "opportunities": [
                    {
                        "symbol": "SPY",
                        "asset_class": "equity_etf",
                        "score": 91.5,
                        "classification": "high_conviction",
                        "timestamp": "2026-04-26T20:00:00Z",
                        "horizon_min": 60,
                        "direction": "long",
                        "snapshot": {"k": "v"},
                        "degradations_active": [],
                        "source": "quant",
                        "trace_id": "trace-f3-1:SPY",
                    }
                ],
            },
            {},
        )

    monkeypatch.setattr(RadarOpportunityClient, "_request_json", _fake_request_json)
    res = client.fetch_opportunities()
    assert res.ok is True
    assert res.trace_id == "trace-f3-1"
    assert len(res.batch.items) == 1
    assert res.batch.items[0].symbol == "SPY"
    assert res.batch.items[0].score == 91.5


def test_radar_client_degrades_on_http_503(monkeypatch: pytest.MonkeyPatch) -> None:
    client = RadarOpportunityClient(
        opportunities_url="http://radar.local/api/radar/opportunities",
        enabled=True,
        timeout_sec=1.0,
    )

    def _raise_http_error(self, _url: str):
        raise HTTPError(
            url="http://radar.local",
            code=503,
            msg="Service Unavailable",
            hdrs={},
            fp=None,
        )

    monkeypatch.setattr(RadarOpportunityClient, "_request_json", _raise_http_error)
    res = client.fetch_opportunities(limit=3, min_score=70)
    assert res.ok is False
    assert res.degraded is True
    assert res.status_code == 503
    assert "radar_http_error_503" in (res.error or "")
