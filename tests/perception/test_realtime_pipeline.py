from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.api.radar import build_realtime_snapshot
from atlas_scanner.perception.market.flow_normalizer import RawFlowEvent, normalize_flow_events
from atlas_scanner.perception.market.openbb_realtime import OpenBBRealtimeAdapter


class _MockOpenBBClient:
    class equity:
        class price:
            @staticmethod
            def historical(symbol: str, provider: str, interval: str):  # noqa: ARG004
                class _Response:
                    @staticmethod
                    def to_df():
                        class _Frame:
                            @staticmethod
                            def to_dict(mode: str):  # noqa: ARG004
                                return [
                                    {"close": 100.0, "volume": 1000.0},
                                    {"close": 101.0, "volume": 1200.0},
                                    {"close": 102.0, "volume": 1300.0},
                                ]

                        return _Frame()

                return _Response()


def test_openbb_realtime_adapter_normalizes_market_snapshot() -> None:
    adapter = OpenBBRealtimeAdapter(obb_client=_MockOpenBBClient(), providers_chain=("polygon",))
    result = adapter.fetch(symbol="SPY", timeframe="1m")
    assert result.snapshot.symbol == "SPY"
    assert result.snapshot.close_price == 102.0
    assert result.snapshot.relative_volume is not None
    assert result.provider_used == "polygon"
    assert result.snapshot.diagnostics.get("provider_status") == "ok"


def test_flow_normalizer_builds_bucketed_distribution() -> None:
    now = datetime.now(timezone.utc)
    events = (
        RawFlowEvent(symbol="SPY", event_ts=now, side="buy", size=100, strike=500.0, dte=1, premium=10000.0, type="call", aggression="aggressive"),
        RawFlowEvent(symbol="SPY", event_ts=now, side="buy", size=90, strike=450.0, dte=10, premium=12000.0, type="put", aggression="neutral"),
    )
    snapshot = normalize_flow_events(symbol="SPY", timeframe="1m", events=events, as_of=now)
    assert snapshot.call_volume == 100
    assert snapshot.put_volume == 90
    assert "0-2" in snapshot.dte_distribution
    assert "8-30" in snapshot.dte_distribution
    assert snapshot.quality_flags.get("has_flow") is True


def test_realtime_snapshot_degrades_when_openbb_unavailable() -> None:
    result = build_realtime_snapshot(
        symbol="QQQ",
        timeframes=("1m", "5m"),
        runtime_mode="paper",
    )
    assert result.batch.signals
    assert any(signal.primary_degradation_reason is not None for signal in result.batch.signals)
    assert result.handoff.metadata.get("source") == "atlas_scanner.inference.handoff"


def test_realtime_snapshot_multitimeframe_consistency() -> None:
    result = build_realtime_snapshot(
        symbol="IWM",
        timeframes=("1m", "5m", "15m", "1h"),
        runtime_mode="paper",
    )
    produced = {signal.timeframe for signal in result.batch.signals}
    assert produced == {"1m", "5m", "15m", "1h"}
