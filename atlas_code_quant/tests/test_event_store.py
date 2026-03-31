"""Tests for learning.event_store — append-only JSONL event sourcing."""
import json
import tempfile
from pathlib import Path

import pytest

from learning.event_store import EventStore


@pytest.fixture
def tmp_store(tmp_path):
    """Create an EventStore backed by a temporary file."""
    return EventStore(path=tmp_path / "events.jsonl", max_memory=100)


def test_append_returns_event(tmp_store):
    ev = tmp_store.append("signal.generated", {"symbol": "SPY", "score": 0.72})
    assert ev["topic"] == "signal.generated"
    assert ev["data"]["symbol"] == "SPY"
    assert ev["seq"] == 0
    assert "ts" in ev


def test_append_increments_seq(tmp_store):
    tmp_store.append("a", {})
    ev2 = tmp_store.append("b", {})
    assert ev2["seq"] == 1


def test_query_by_exact_topic(tmp_store):
    tmp_store.append("signal.generated", {"symbol": "SPY"})
    tmp_store.append("order.submitted", {"symbol": "AAPL"})
    results = tmp_store.query(topic="signal.generated")
    assert len(results) == 1
    assert results[0]["data"]["symbol"] == "SPY"


def test_query_wildcard_topic(tmp_store):
    tmp_store.append("signal.generated", {})
    tmp_store.append("signal.blocked", {})
    tmp_store.append("order.submitted", {})
    results = tmp_store.query(topic="signal.*")
    assert len(results) == 2


def test_query_by_source(tmp_store):
    tmp_store.append("a", {}, source="auto_cycle")
    tmp_store.append("b", {}, source="manual")
    results = tmp_store.query(source="auto_cycle")
    assert len(results) == 1


def test_query_limit(tmp_store):
    for i in range(10):
        tmp_store.append("tick", {"i": i})
    results = tmp_store.query(limit=3)
    assert len(results) == 3


def test_flush_persists_to_disk(tmp_store):
    for i in range(5):
        tmp_store.append("test", {"i": i})
    count = tmp_store.flush()
    assert count == 5
    lines = tmp_store._path.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 5
    first = json.loads(lines[0])
    assert first["topic"] == "test"


def test_auto_flush_at_threshold(tmp_path):
    store = EventStore(path=tmp_path / "events.jsonl", max_memory=200)
    # _FLUSH_EVERY is 50 — after 50 appends, should auto-flush
    for i in range(55):
        store.append("tick", {"i": i})
    assert store._path.exists()
    lines = store._path.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 50  # first 50 auto-flushed


def test_reload_from_disk(tmp_path):
    path = tmp_path / "events.jsonl"
    store1 = EventStore(path=path, max_memory=100)
    for i in range(10):
        store1.append("tick", {"i": i})
    store1.flush()

    # New store loads from same file
    store2 = EventStore(path=path, max_memory=100)
    results = store2.query(topic="tick")
    assert len(results) == 10


def test_stats(tmp_store):
    tmp_store.append("signal.generated", {})
    tmp_store.append("signal.generated", {})
    tmp_store.append("order.submitted", {})
    stats = tmp_store.stats()
    assert stats["total_appended"] == 3
    assert stats["buffer_size"] == 3
    assert stats["topic_counts"]["signal.generated"] == 2


def test_clear(tmp_store):
    tmp_store.append("a", {})
    tmp_store.flush()
    tmp_store.clear()
    assert tmp_store.stats()["buffer_size"] == 0
    assert not tmp_store._path.exists()


def test_replay_returns_all(tmp_store):
    for i in range(5):
        tmp_store.append("tick", {"i": i})
    events = tmp_store.replay()
    assert len(events) == 5
