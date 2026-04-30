"""Evaluación periódica del BrainCore (cola de eventos)."""
from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..brain.brain_core import BrainCore

logger = logging.getLogger("atlas.brain.event_loop")


def run_event_loop(
    brain: BrainCore,
    stop_event: threading.Event,
    interval_sec: float = 1.0,
) -> None:
    while not stop_event.is_set():
        queue_before = brain.pending_event_count()
        try:
            brain.evaluate()
        except Exception as exc:
            logger.exception("brain.evaluate error: %s", exc)
        finally:
            queue_after = brain.pending_event_count()
            brain.note_runtime_tick(queue_before, queue_after)
        if stop_event.wait(interval_sec):
            break


def start_event_loop_background(
    brain: BrainCore,
    stop_event: threading.Event,
    interval_sec: float = 1.0,
) -> threading.Thread:
    t = threading.Thread(
        target=run_event_loop,
        args=(brain, stop_event, interval_sec),
        name="atlas-brain-event",
        daemon=True,
    )
    t.start()
    return t
