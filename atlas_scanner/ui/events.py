from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import os
from threading import Lock
from typing import Any

from atlas_scanner.ui.backplane.redis import RedisBackplane, RedisBackplaneUnavailable
from atlas_scanner.ui.stream_types import RadarStreamEvent


class EventBackend:
    backend_type: str = "memory"

    def publish(self, *, event_type: str, payload: dict[str, Any]) -> RadarStreamEvent:
        raise NotImplementedError

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        raise NotImplementedError

    def latest_seq(self) -> int:
        raise NotImplementedError

    def metrics(self) -> dict[str, Any]:
        raise NotImplementedError


class InMemoryBackend(EventBackend):
    backend_type = "memory"

    def __init__(self, *, max_events: int = 1024) -> None:
        self._events: deque[RadarStreamEvent] = deque(maxlen=max(100, int(max_events)))
        self._seq = 0
        self._lock = Lock()
        self._published_count = 0
        self._dropped_count = 0
        self._last_error: str | None = None

    def publish(self, *, event_type: str, payload: dict[str, Any]) -> RadarStreamEvent:
        now = datetime.now(timezone.utc).isoformat()
        with self._lock:
            self._seq += 1
            event = RadarStreamEvent(
                seq=self._seq,
                event_type=event_type,
                payload={**payload, "timestamp": payload.get("timestamp") or now},
                timestamp=now,
            )
            self._events.append(event)
            self._published_count += 1
            return event

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        with self._lock:
            return tuple(event for event in self._events if event.seq > last_seq)

    def latest_seq(self) -> int:
        with self._lock:
            return self._seq

    def metrics(self) -> dict[str, Any]:
        return {
            "status": "connected",
            "published_count": self._published_count,
            "dropped_count": self._dropped_count,
            "last_error": self._last_error,
            "buffer_size": self._events.maxlen,
            "latest_seq": self._seq,
        }


class RedisBackend(EventBackend):
    backend_type = "redis"

    def __init__(self, *, redis_url: str, channel: str, max_events: int = 1024) -> None:
        self._inner = RedisBackplane(redis_url=redis_url, channel=channel, max_events=max_events)

    def publish(self, *, event_type: str, payload: dict[str, Any]) -> RadarStreamEvent:
        return self._inner.publish(event_type=event_type, payload=payload)

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        return self._inner.since(last_seq)

    def latest_seq(self) -> int:
        return self._inner.latest_seq()

    def metrics(self) -> dict[str, Any]:
        return self._inner.metrics()


class RadarEventBus:
    def __init__(self, *, max_events: int = 1024, backend: EventBackend | None = None) -> None:
        self._backend = backend or InMemoryBackend(max_events=max_events)
        self._fallback_backend = InMemoryBackend(max_events=max_events)
        self._fallback_active = False
        self._fallback_reason: str | None = None

    def publish(self, *, event_type: str, payload: dict[str, Any]) -> RadarStreamEvent:
        if self._fallback_active:
            return self._fallback_backend.publish(event_type=event_type, payload=payload)
        try:
            return self._backend.publish(event_type=event_type, payload=payload)
        except Exception as exc:
            self._fallback_active = True
            self._fallback_reason = str(exc)
            return self._fallback_backend.publish(event_type=event_type, payload=payload)

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        if self._fallback_active:
            return self._fallback_backend.since(last_seq)
        try:
            return self._backend.since(last_seq)
        except Exception as exc:
            self._fallback_active = True
            self._fallback_reason = str(exc)
            return self._fallback_backend.since(last_seq)

    def latest_seq(self) -> int:
        if self._fallback_active:
            return self._fallback_backend.latest_seq()
        try:
            return self._backend.latest_seq()
        except Exception as exc:
            self._fallback_active = True
            self._fallback_reason = str(exc)
            return self._fallback_backend.latest_seq()

    def metrics(self) -> dict[str, Any]:
        return {
            "active_backend": "memory" if self._fallback_active else self._backend.backend_type,
            "configured_backend": self._backend.backend_type,
            "fallback_active": self._fallback_active,
            "fallback_reason": self._fallback_reason,
            "backend_metrics": self._backend.metrics(),
            "fallback_metrics": self._fallback_backend.metrics() if self._fallback_active else None,
        }


def event_bus_from_env() -> RadarEventBus:
    buffer_size = max(100, int(os.getenv("ATLAS_RADAR_BACKPLANE_BUFFER_SIZE", "1024")))
    backend_type = (os.getenv("ATLAS_RADAR_BACKPLANE_TYPE", "memory").strip().lower() or "memory")
    if backend_type == "redis":
        redis_url = os.getenv("ATLAS_RADAR_REDIS_URL", "").strip()
        channel = os.getenv("ATLAS_RADAR_REDIS_CHANNEL", "atlas:radar:stream").strip() or "atlas:radar:stream"
        try:
            return RadarEventBus(
                max_events=buffer_size,
                backend=RedisBackend(redis_url=redis_url, channel=channel, max_events=buffer_size),
            )
        except RedisBackplaneUnavailable:
            return RadarEventBus(max_events=buffer_size, backend=InMemoryBackend(max_events=buffer_size))
    return RadarEventBus(max_events=buffer_size, backend=InMemoryBackend(max_events=buffer_size))


def to_sse_frame(event: RadarStreamEvent) -> str:
    body = json.dumps(
        {
            "seq": event.seq,
            "type": event.event_type,
            "timestamp": event.timestamp,
            "payload": event.payload,
        },
        ensure_ascii=False,
    )
    return f"id: {event.seq}\nevent: {event.event_type}\ndata: {body}\n\n"
