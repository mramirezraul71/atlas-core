"""Polling periódico de adapters → StateBus."""
from __future__ import annotations

import logging
import threading
import time
from datetime import datetime, timezone
from typing import Any

from ..brain.state_bus import StateBus

logger = logging.getLogger("atlas.brain.heartbeat")


def run_heartbeat_loop(
    adapters: dict[str, Any],
    state_bus: StateBus,
    stop_event: threading.Event,
    interval_sec: float = 2.0,
) -> None:
    last_beat = time.monotonic()
    while not stop_event.is_set():
        beat_started = state_bus.mark_loop_start("heartbeat")
        for name, adapter in adapters.items():
            try:
                st = adapter.get_state()
                rk = adapter.get_risks()
                state_bus.update_module_state(st.name, st)
                state_bus.update_risk(rk.name, rk)
            except Exception as exc:
                logger.warning("heartbeat adapter %s: %s", name, exc)
        now = time.monotonic()
        state_bus.increment_meta_counter("heartbeat_count", 1)
        state_bus.set_meta("last_heartbeat_monotonic", now)
        state_bus.set_meta("last_heartbeat_ts_utc", datetime.now(timezone.utc).isoformat())
        state_bus.set_meta("heartbeat_interval_sec", max(0.0, now - last_beat))
        state_bus.mark_loop_end("heartbeat", beat_started)
        last_beat = now
        if stop_event.wait(interval_sec):
            break


def start_heartbeat_background(
    adapters: dict[str, Any],
    state_bus: StateBus,
    stop_event: threading.Event,
    interval_sec: float = 2.0,
) -> threading.Thread:
    t = threading.Thread(
        target=run_heartbeat_loop,
        args=(adapters, state_bus, stop_event, interval_sec),
        name="atlas-brain-heartbeat",
        daemon=True,
    )
    t.start()
    return t
