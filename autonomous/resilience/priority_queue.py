"""
PriorityQueue - Cola con prioridades CRITICAL, HIGH, MEDIUM, LOW.
Tareas críticas (health, healing) se procesan primero.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Any, Callable

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class Priority(IntEnum):
    CRITICAL = 0
    HIGH = 1
    MEDIUM = 2
    LOW = 3


@dataclass
class QueuedTask:
    priority: Priority
    payload: Any
    metadata: dict
    created_at: float = field(default_factory=time.time)


class PriorityQueue:
    """
    enqueue(task, priority, metadata); dequeue() → mayor prioridad;
    get_queue_stats(); reorder_on_emergency() pone CRITICAL primero.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("resilience", {})
        self._max_size = int(self._config.get("priority_queue", {}).get("max_queue_size", 1000))
        self._lock = threading.Lock()
        self._queues: dict[Priority, list[QueuedTask]] = {
            Priority.CRITICAL: [],
            Priority.HIGH: [],
            Priority.MEDIUM: [],
            Priority.LOW: [],
        }

    def enqueue(self, task: Any, priority: Priority | str = Priority.MEDIUM, metadata: dict | None = None) -> bool:
        """Añade tarea a la cola. False si cola llena."""
        if isinstance(priority, str):
            priority = getattr(Priority, priority.upper(), Priority.MEDIUM)
        with self._lock:
            total = sum(len(q) for q in self._queues.values())
            if total >= self._max_size:
                return False
            self._queues[priority].append(QueuedTask(priority=priority, payload=task, metadata=metadata or {}))
            return True

    def dequeue(self) -> QueuedTask | None:
        """Saca la tarea de mayor prioridad (CRITICAL primero)."""
        with self._lock:
            for p in Priority:
                if self._queues[p]:
                    return self._queues[p].pop(0)
            return None

    def get_queue_stats(self) -> dict[str, int]:
        """Tamaños por prioridad."""
        with self._lock:
            return {p.name: len(self._queues[p]) for p in Priority}

    def reorder_on_emergency(self) -> None:
        """Mueve todas las CRITICAL al frente (ya están en cola separada; no reordena entre prioridades)."""
        pass
