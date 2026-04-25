from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from threading import Lock
from typing import Any


@dataclass(frozen=True)
class RadarStreamEvent:
    seq: int
    event_type: str
    payload: dict[str, Any]
    timestamp: str


class RadarEventBus:
    def __init__(self, *, max_events: int = 1024) -> None:
        self._events: deque[RadarStreamEvent] = deque(maxlen=max(100, int(max_events)))
        self._seq = 0
        self._lock = Lock()

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
            return event

    def since(self, last_seq: int) -> tuple[RadarStreamEvent, ...]:
        with self._lock:
            return tuple(event for event in self._events if event.seq > last_seq)

    def latest_seq(self) -> int:
        with self._lock:
            return self._seq


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
