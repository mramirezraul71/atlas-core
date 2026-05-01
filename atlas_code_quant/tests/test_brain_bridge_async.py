from __future__ import annotations

import time
from pathlib import Path
from threading import Event

from atlas_code_quant.operations.brain_bridge import QuantBrainBridge


def test_emit_queues_bitacora_without_blocking_main_thread(tmp_path: Path, monkeypatch) -> None:
    state_path = tmp_path / "quant_brain_bridge_state.json"
    events_path = tmp_path / "quant_brain_bridge.jsonl"
    bridge = QuantBrainBridge(
        base_url="http://127.0.0.1:8791",
        state_path=state_path,
        events_path=events_path,
    )
    bridge.enabled = True
    bridge.bitacora_enabled = True
    bridge.memory_enabled = False

    delivered = Event()
    calls: list[tuple[str, dict[str, object]]] = []

    def _slow_post(path: str, payload: dict[str, object]) -> dict[str, object]:
        time.sleep(0.8)
        calls.append((path, payload))
        delivered.set()
        return {"ok": True}

    monkeypatch.setattr(bridge, "_post_json", _slow_post)

    started = time.perf_counter()
    payload = bridge.emit(
        kind="unit_test",
        message="bridge async",
        level="info",
        memorize=False,
    )
    elapsed = time.perf_counter() - started

    assert elapsed < 0.35
    assert payload["bitacora_queued"] is True
    assert delivered.wait(2.0) is True
    assert calls and calls[0][0] == "/bitacora/log"
    status = bridge.status()
    assert status["last_bitacora_ok"] is True
    assert status["pending_replay_events"] == 0
    assert status["queued_local_only_events"] == 0
    assert status["delivered_events"] == 1


def test_flush_pending_events_replays_memory_backlog(tmp_path: Path, monkeypatch) -> None:
    state_path = tmp_path / "quant_brain_bridge_state.json"
    events_path = tmp_path / "quant_brain_bridge.jsonl"
    bridge = QuantBrainBridge(
        base_url="http://127.0.0.1:8791",
        state_path=state_path,
        events_path=events_path,
    )
    bridge.enabled = True
    bridge.bitacora_enabled = False
    bridge.memory_enabled = True

    attempts = {"count": 0}

    def _memory_post(path: str, payload: dict[str, object], *, timeout_sec: int | None = None) -> dict[str, object]:
        attempts["count"] += 1
        if attempts["count"] == 1:
            raise TimeoutError("memory offline")
        return {"ok": True}

    monkeypatch.setattr(bridge, "_post_query", _memory_post)

    payload = bridge.emit(
        kind="unit_test",
        message="bridge replay",
        level="info",
        memorize=True,
    )

    assert payload["ok"] is False
    assert bridge.status()["pending_replay_events"] == 1
    assert bridge.status()["delivered_events"] == 0

    replay = bridge.flush_pending_events()
    status = bridge.status()

    assert replay["ok"] is True
    assert replay["resolved"] == 1
    assert status["pending_replay_events"] == 0
    assert status["queued_local_only_events"] == 0
    assert status["delivered_events"] == 1


def test_status_migrates_legacy_local_only_events(tmp_path: Path) -> None:
    state_path = tmp_path / "quant_brain_bridge_state.json"
    events_path = tmp_path / "quant_brain_bridge.jsonl"
    state_path.write_text(
        '{"total_events": 10, "delivered_events": 6, "queued_local_only_events": 4, "last_memory_ok": true, "last_bitacora_ok": true}',
        encoding="utf-8",
    )
    bridge = QuantBrainBridge(
        base_url="http://127.0.0.1:8791",
        state_path=state_path,
        events_path=events_path,
    )

    status = bridge.status()

    assert status["delivery_tracking_version"] == 2
    assert status["historical_local_only_events"] == 4
    assert status["pending_replay_events"] == 0
    assert status["queued_local_only_events"] == 0
