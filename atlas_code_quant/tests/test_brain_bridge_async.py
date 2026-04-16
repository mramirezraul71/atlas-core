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
    assert bridge.status()["last_bitacora_ok"] is True
