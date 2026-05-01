"""Lightweight event sourcing store for trading signals and decisions.

Persists the last N events to a DuckDB-backed Parquet file so they can be
replayed, queried, and analyzed.  Designed to sit alongside the existing
EventBus without modifying its core — just subscribe and record.

Usage:
    store = EventStore()
    store.append("signal.generated", {"symbol": "SPY", "score": 0.72, ...})
    store.append("order.submitted", {"symbol": "SPY", "side": "buy", ...})

    # Query last 100 signal events
    events = store.query(topic="signal.*", limit=100)

    # Replay to rebuild state
    for ev in store.replay(since="2026-03-30T00:00:00"):
        process(ev)
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger("atlas.event_store")

_DEFAULT_PATH = Path(os.environ.get(
    "ATLAS_EVENT_STORE_PATH",
    str(Path(__file__).resolve().parent.parent / "data" / "learning" / "event_store.jsonl"),
))

_MAX_MEMORY = 10_000
_FLUSH_EVERY = 50


class EventStore:
    """Append-only event store with JSONL persistence and in-memory buffer."""

    def __init__(self, path: Path | None = None, max_memory: int = _MAX_MEMORY):
        self._path = path or _DEFAULT_PATH
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._buffer: deque[dict] = deque(maxlen=max_memory)
        self._lock = threading.Lock()
        self._pending: list[dict] = []
        self._total_appended = 0
        self._load_existing()

    def _load_existing(self) -> None:
        """Load recent events from disk into memory buffer at startup."""
        if not self._path.exists():
            return
        try:
            with open(self._path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            # Only load last max_memory lines
            for line in lines[-self._buffer.maxlen:]:
                line = line.strip()
                if not line:
                    continue
                try:
                    self._buffer.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
            logger.info("EventStore loaded %d events from disk", len(self._buffer))
        except Exception as e:
            logger.warning("EventStore failed to load: %s", e)

    def append(self, topic: str, data: dict[str, Any] | None = None,
               source: str = "quant") -> dict:
        """Record a new event.

        Args:
            topic: Event topic (e.g. "signal.generated", "order.filled").
            data: Arbitrary event payload.
            source: Origin module identifier.

        Returns:
            The created event record.
        """
        event = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "seq": self._total_appended,
            "topic": topic,
            "source": source,
            "data": data or {},
        }

        with self._lock:
            self._buffer.append(event)
            self._pending.append(event)
            self._total_appended += 1

            if len(self._pending) >= _FLUSH_EVERY:
                self._flush_unlocked()

        return event

    def flush(self) -> int:
        """Force flush pending events to disk."""
        with self._lock:
            return self._flush_unlocked()

    def _flush_unlocked(self) -> int:
        if not self._pending:
            return 0
        try:
            with open(self._path, "a", encoding="utf-8") as f:
                for ev in self._pending:
                    f.write(json.dumps(ev, ensure_ascii=False, default=str) + "\n")
            count = len(self._pending)
            self._pending.clear()
            return count
        except Exception as e:
            logger.error("EventStore flush failed: %s", e)
            return 0

    def query(self, *, topic: str | None = None, source: str | None = None,
              since: str | None = None, limit: int = 100) -> list[dict]:
        """Query events from the in-memory buffer.

        Args:
            topic: Filter by topic prefix (supports "signal.*" style matching).
            source: Filter by source module.
            since: ISO timestamp — only events after this time.
            limit: Max results to return.
        """
        results = []
        prefix = topic.rstrip("*").rstrip(".") if topic and "*" in topic else None

        with self._lock:
            for ev in reversed(self._buffer):
                if limit and len(results) >= limit:
                    break
                if topic and not prefix and ev["topic"] != topic:
                    continue
                if prefix and not ev["topic"].startswith(prefix):
                    continue
                if source and ev.get("source") != source:
                    continue
                if since and ev["ts"] < since:
                    break  # events are chronological, can stop early
                results.append(ev)

        results.reverse()
        return results

    def replay(self, since: str | None = None, topic: str | None = None) -> list[dict]:
        """Replay all events since a given timestamp — for state reconstruction."""
        return self.query(topic=topic, since=since, limit=0)

    def stats(self) -> dict[str, Any]:
        """Return store statistics."""
        with self._lock:
            topics: dict[str, int] = {}
            for ev in self._buffer:
                topics[ev["topic"]] = topics.get(ev["topic"], 0) + 1
            return {
                "total_appended": self._total_appended,
                "buffer_size": len(self._buffer),
                "buffer_max": self._buffer.maxlen,
                "pending_flush": len(self._pending),
                "disk_path": str(self._path),
                "disk_exists": self._path.exists(),
                "disk_size_kb": round(self._path.stat().st_size / 1024, 1)
                    if self._path.exists() else 0,
                "topic_counts": dict(sorted(topics.items(), key=lambda x: -x[1])[:20]),
            }

    def clear(self) -> None:
        """Clear all events (memory + disk). Use with caution."""
        with self._lock:
            self._buffer.clear()
            self._pending.clear()
            self._total_appended = 0
            if self._path.exists():
                self._path.unlink()


# Singleton
_store: EventStore | None = None


def get_event_store() -> EventStore:
    global _store
    if _store is None:
        _store = EventStore()
    return _store
