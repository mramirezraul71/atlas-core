from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import RadarDecisionHandoff
from atlas_scanner.inference.handoff_registry import HandoffRegistry, InMemoryHandoffConsumer


def test_handoff_registry_publish_to_stub_consumer() -> None:
    registry = HandoffRegistry()
    consumer = InMemoryHandoffConsumer()
    registry.register("stub", consumer)
    handoff = RadarDecisionHandoff(
        symbol="SPY",
        as_of=datetime.now(timezone.utc),
        operable=True,
        primary_timeframe="1m",
        primary_signal=None,
        handoff_summary="test",
    )
    delivered = registry.publish(handoff)
    assert delivered == ("stub",)
    assert len(consumer.handoffs) == 1
    assert consumer.handoffs[0].symbol == "SPY"
