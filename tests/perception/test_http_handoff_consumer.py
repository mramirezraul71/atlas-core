from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarDecisionHandoff
import atlas_scanner.inference.http_handoff_consumer as http_consumer


class _MockResponse:
    def __init__(self, status_code: int):
        self.status_code = status_code


def test_http_handoff_consumer_posts_payload(monkeypatch) -> None:
    calls: list[dict] = []

    def _fake_post(url, data, headers, timeout):  # noqa: ANN001
        calls.append({"url": url, "data": data, "headers": headers, "timeout": timeout})
        return _MockResponse(200)

    monkeypatch.setattr(http_consumer.requests, "post", _fake_post)
    consumer = http_consumer.HttpHandoffConsumer(
        config=http_consumer.HttpHandoffConfig(endpoint_url="https://example.local/handoff")
    )
    handoff = RadarDecisionHandoff(
        symbol="SPY",
        as_of=datetime.now(timezone.utc),
        operable=True,
        primary_timeframe="1m",
        primary_signal=None,
        handoff_summary="ok",
    )
    consumer.consume(handoff)
    assert len(calls) == 1
    assert calls[0]["headers"]["Idempotency-Key"]


def test_http_handoff_consumer_retries_then_gives_up(monkeypatch) -> None:
    attempts = {"n": 0}

    def _fake_post(url, data, headers, timeout):  # noqa: ANN001
        _ = url, data, headers, timeout
        attempts["n"] += 1
        return _MockResponse(503)

    monkeypatch.setattr(http_consumer.requests, "post", _fake_post)
    consumer = http_consumer.HttpHandoffConsumer(
        config=http_consumer.HttpHandoffConfig(
            endpoint_url="https://example.local/handoff",
            retries=2,
            backoff_sec=0.0,
        )
    )
    handoff = RadarDecisionHandoff(
        symbol="QQQ",
        as_of=datetime.now(timezone.utc),
        operable=False,
        primary_timeframe=None,
        primary_signal=None,
        handoff_summary="fail",
    )
    consumer.consume(handoff)
    assert attempts["n"] == 3
