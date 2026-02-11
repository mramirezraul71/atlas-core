"""In-memory event dispatch."""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional


class EventBus:
    """Subscribe by topic, publish events."""

    def __init__(self) -> None:
        self._handlers: Dict[str, List[Callable[..., None]]] = {}

    def subscribe(self, topic: str, handler: Callable[..., None]) -> None:
        self._handlers.setdefault(topic, []).append(handler)

    def publish(self, topic: str, *args: Any, **kwargs: Any) -> None:
        for h in self._handlers.get(topic, []):
            try:
                h(*args, **kwargs)
            except Exception:
                pass

    def clear(self, topic: Optional[str] = None) -> None:
        if topic is None:
            self._handlers.clear()
        else:
            self._handlers.pop(topic, None)
