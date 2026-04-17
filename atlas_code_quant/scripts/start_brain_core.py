"""Arranque del ATLAS Brain Core (heartbeat + event loop)."""
from __future__ import annotations

import logging
import signal
import sys
import threading
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)

logger = logging.getLogger("atlas.brain.startup")


def main() -> None:
    from atlas_core.runtime.bootstrap import build_brain_system
    from atlas_core.runtime.event_loop import start_event_loop_background
    from atlas_core.runtime.heartbeat_loop import start_heartbeat_background

    rt = build_brain_system()
    stop = threading.Event()

    def _stop(*_: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    start_heartbeat_background(rt.adapters, rt.state_bus, stop, interval_sec=2.0)
    start_event_loop_background(rt.brain, stop, interval_sec=1.0)

    logger.info("ATLAS Brain Core activo (heartbeat + event loop). Ctrl+C para salir.")
    try:
        while not stop.is_set():
            stop.wait(0.5)
    except KeyboardInterrupt:
        stop.set()
    logger.info("Brain Core detenido.")


if __name__ == "__main__":
    main()
