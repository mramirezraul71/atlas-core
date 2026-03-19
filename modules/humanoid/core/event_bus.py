"""Central reusable event bus for hybrid event-driven ATLAS operations.

This bus is intentionally lightweight:
- local singleton for low-latency module-to-module signaling
- optional forwarding into the humanoid kernel event bus when the kernel
  already exists, preserving compatibility with current event consumers
- no startup side effects: it never forces kernel creation
"""
from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Dict, Optional

from modules.humanoid.kernel.event_bus import EventBus

_log = logging.getLogger("atlas.core.event_bus")
_BUS: Optional[EventBus] = None
_LOCK = threading.Lock()
_HANDLERS_BOOTSTRAPPED = False


def get_event_bus() -> EventBus:
    """Return the central in-process event bus singleton."""
    global _BUS
    if _BUS is None:
        with _LOCK:
            if _BUS is None:
                _BUS = EventBus(log_events=False)
    _bootstrap_default_handlers()
    return _BUS


def _bootstrap_default_handlers() -> None:
    global _HANDLERS_BOOTSTRAPPED
    if _HANDLERS_BOOTSTRAPPED:
        return
    with _LOCK:
        if _HANDLERS_BOOTSTRAPPED:
            return
        try:
            from .event_handlers import register_default_event_handlers

            register_default_event_handlers(_BUS or EventBus(log_events=False))
        except Exception as exc:
            _log.debug("Default event handler bootstrap skipped: %s", exc)
        _HANDLERS_BOOTSTRAPPED = True


def _try_forward_to_kernel(topic: str, payload: Dict[str, Any]) -> None:
    """Forward to the humanoid kernel only if it already exists."""
    try:
        import modules.humanoid as humanoid_module

        kernel = getattr(humanoid_module, "_humanoid_kernel", None)
        if kernel is None:
            return
        kernel.dispatch(topic, payload)
    except Exception as exc:
        _log.debug("Kernel event forward skipped for %s: %s", topic, exc)


def publish_event(
    topic: str,
    payload: Optional[Dict[str, Any]] = None,
    *,
    forward_to_kernel: bool = True,
    **kwargs: Any,
) -> Dict[str, Any]:
    """Publish an operational event on the central bus.

    `kwargs` are merged into `payload` to keep the callsites compact.
    """
    data = dict(payload or {})
    data.update(kwargs)

    result = get_event_bus().publish(topic, data)
    if forward_to_kernel:
        _try_forward_to_kernel(topic, data)
    return result


def subscribe_event(
    topic: str,
    handler: Callable[..., Any],
    *,
    priority: int = 0,
    once: bool = False,
) -> Callable[[], None]:
    """Subscribe a handler to the central event bus."""
    return get_event_bus().subscribe(topic, handler, priority=priority, once=once)


__all__ = ["get_event_bus", "publish_event", "subscribe_event"]
