from __future__ import annotations

import sys
import types

from atlas_scanner.ui.events import EventBackend, RadarEventBus, event_bus_from_env


class _FakePubSub:
    def __init__(self, queue_ref):
        self._queue_ref = queue_ref

    def subscribe(self, _channel: str) -> None:
        return None

    def get_message(self, *, ignore_subscribe_messages: bool = True, timeout: float = 1.0):
        _ = ignore_subscribe_messages
        _ = timeout
        if not self._queue_ref:
            return None
        return {"data": self._queue_ref.pop(0)}


class _FakeRedisClient:
    def __init__(self) -> None:
        self.seq = 0
        self.queue: list[str] = []

    def incr(self, _key: str) -> int:
        self.seq += 1
        return self.seq

    def publish(self, _channel: str, payload: str) -> int:
        self.queue.append(payload)
        return 1

    def pubsub(self) -> _FakePubSub:
        return _FakePubSub(self.queue)


class _FailBackend(EventBackend):
    backend_type = "redis"

    def publish(self, *, event_type: str, payload: dict):
        _ = event_type
        _ = payload
        raise RuntimeError("forced_publish_failure")

    def since(self, last_seq: int):
        _ = last_seq
        raise RuntimeError("forced_since_failure")

    def latest_seq(self) -> int:
        raise RuntimeError("forced_latest_failure")

    def metrics(self) -> dict:
        return {"status": "degraded"}


def test_event_bus_uses_memory_backend_by_default(monkeypatch) -> None:
    monkeypatch.delenv("ATLAS_RADAR_BACKPLANE_TYPE", raising=False)
    bus = event_bus_from_env()
    event = bus.publish(event_type="heartbeat", payload={"status": "alive"})
    assert event.seq >= 1
    metrics = bus.metrics()
    assert metrics["active_backend"] == "memory"


def test_event_bus_uses_redis_backend_when_available(monkeypatch) -> None:
    fake_client = _FakeRedisClient()

    class _FakeRedisClass:
        @staticmethod
        def from_url(_url: str, decode_responses: bool = True):
            _ = decode_responses
            return fake_client

    fake_module = types.SimpleNamespace(Redis=_FakeRedisClass)
    monkeypatch.setitem(sys.modules, "redis", fake_module)
    monkeypatch.setenv("ATLAS_RADAR_BACKPLANE_TYPE", "redis")
    monkeypatch.setenv("ATLAS_RADAR_REDIS_URL", "redis://fake:6379/0")
    monkeypatch.setenv("ATLAS_RADAR_REDIS_CHANNEL", "atlas:test")
    bus = event_bus_from_env()
    event = bus.publish(event_type="snapshot", payload={"symbol": "SPY"})
    assert event.seq == 1
    assert "snapshot" in fake_client.queue[0]
    metrics = bus.metrics()
    assert metrics["configured_backend"] == "redis"


def test_event_bus_falls_back_to_memory_when_backend_fails() -> None:
    bus = RadarEventBus(backend=_FailBackend(), max_events=100)
    event = bus.publish(event_type="alert", payload={"message": "fallback"})
    assert event.seq == 1
    metrics = bus.metrics()
    assert metrics["fallback_active"] is True
    assert metrics["active_backend"] == "memory"


def test_event_bus_env_redis_without_url_falls_back_to_memory(monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_RADAR_BACKPLANE_TYPE", "redis")
    monkeypatch.setenv("ATLAS_RADAR_REDIS_URL", "")
    bus = event_bus_from_env()
    event = bus.publish(event_type="heartbeat", payload={"status": "alive"})
    assert event.seq >= 1
    assert bus.metrics()["active_backend"] == "memory"
