from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import json
import threading
import time
from typing import Any, Callable

from atlas_scanner.ui.stream_types import RadarStreamEvent


class RedisBackplaneUnavailable(RuntimeError):
    """Raised when redis backend cannot be initialized."""


@dataclass
class RedisBackplaneMetrics:
    status: str = "initializing"
    published_count: int = 0
    dropped_count: int = 0
    last_error: str | None = None
    last_publish_latency_ms: float = 0.0
    last_message_at: str | None = None
    listener_running: bool = False


class RedisBackplane:
    def __init__(
        self,
        *,
        redis_url: str,
        channel: str,
        max_events: int,
        redis_client_factory: Callable[[], Any] | None = None,
    ) -> None:
        if not redis_url:
            raise RedisBackplaneUnavailable("missing_redis_url")
        self._redis_url = redis_url
        self._channel = channel
        self._seq_key = f"{channel}:seq"
        self._events: deque[RadarStreamEvent] = deque(maxlen=max(100, int(max_events)))
        self._lock = threading.Lock()
        self._metrics = RedisBackplaneMetrics()
        self._stop_event = threading.Event()
        self._redis_client_factory = redis_client_factory
        self._client = self._build_client()
        self._pubsub = self._client.pubsub()
        try:
            self._pubsub.subscribe(self._channel)
        except Exception as exc:  # pragma: no cover - defensive network handling
            raise RedisBackplaneUnavailable(f"redis_subscribe_failed:{exc}") from exc

        self._listener_thread = threading.Thread(target=self._listen_forever, name="radar-redis-backplane", daemon=True)
        self._listener_thread.start()
        self._metrics.status = "connected"
        self._metrics.listener_running = True

    def publish(self, *, event_type: str, payload: dict[str, Any]) -> RadarStreamEvent:
        started = time.perf_counter()
        try:
            seq = int(self._client.incr(self._seq_key))
            now = datetime.now(timezone.utc).isoformat()
            envelope = {
                "seq": seq,
                "event_type": event_type,
                "timestamp": now,
                "payload": {**payload, "timestamp": payload.get("timestamp") or now},
            }
            self._client.publish(self._channel, json.dumps(envelope, ensure_ascii=False))
            event = RadarStreamEvent(
                seq=seq,
                event_type=event_type,
                payload=dict(envelope["payload"]),
                timestamp=now,
            )
            self._append_event(event)
            self._metrics.published_count += 1
            self._metrics.status = "connected"
            return event
        except Exception as exc:
            self._metrics.dropped_count += 1
            self._metrics.last_error = str(exc)
            self._metrics.status = "degraded"
            raise
        finally:
            self._metrics.last_publish_latency_ms = round((time.perf_counter() - started) * 1000.0, 3)

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        with self._lock:
            return tuple(event for event in self._events if event.seq > last_seq)

    def latest_seq(self) -> int:
        with self._lock:
            if not self._events:
                return 0
            return self._events[-1].seq

    def metrics(self) -> dict[str, Any]:
        return {
            "status": self._metrics.status,
            "published_count": self._metrics.published_count,
            "dropped_count": self._metrics.dropped_count,
            "last_error": self._metrics.last_error,
            "last_publish_latency_ms": self._metrics.last_publish_latency_ms,
            "last_message_at": self._metrics.last_message_at,
            "listener_running": self._metrics.listener_running,
            "channel": self._channel,
            "redis_url": self._redis_url,
        }

    def _build_client(self) -> Any:
        if self._redis_client_factory is not None:
            return self._redis_client_factory()
        try:
            redis_mod = importlib.import_module("redis")
        except Exception as exc:
            raise RedisBackplaneUnavailable(f"redis_module_unavailable:{exc}") from exc
        return redis_mod.Redis.from_url(self._redis_url, decode_responses=True)

    def _listen_forever(self) -> None:
        while not self._stop_event.is_set():
            try:
                message = self._pubsub.get_message(ignore_subscribe_messages=True, timeout=1.0)
                if not message:
                    continue
                payload_raw = message.get("data")
                if not payload_raw:
                    continue
                if isinstance(payload_raw, bytes):
                    payload_raw = payload_raw.decode("utf-8")
                envelope = json.loads(str(payload_raw))
                event = RadarStreamEvent(
                    seq=int(envelope.get("seq", 0)),
                    event_type=str(envelope.get("event_type", "unknown")),
                    payload=dict(envelope.get("payload") or {}),
                    timestamp=str(envelope.get("timestamp") or datetime.now(timezone.utc).isoformat()),
                )
                self._append_event(event)
                self._metrics.last_message_at = datetime.now(timezone.utc).isoformat()
                self._metrics.status = "connected"
            except Exception as exc:  # pragma: no cover - defensive network handling
                self._metrics.last_error = str(exc)
                self._metrics.status = "degraded"
                time.sleep(0.5)

    def _append_event(self, event: RadarStreamEvent) -> None:
        with self._lock:
            if self._events and event.seq <= self._events[-1].seq:
                return
            self._events.append(event)
