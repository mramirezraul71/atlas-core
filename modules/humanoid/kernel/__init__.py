"""Humanoid kernel: registry, event bus, health."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseModule, HealthCheckMixin
from .event_bus import EventBus
from .health import HealthMonitor, health_all
from .registry import ModuleRegistry


class Kernel:
    """Central kernel: registry + event bus, runs health checks."""

    def __init__(self) -> None:
        self.registry = ModuleRegistry()
        self.events = EventBus()
        self.health_monitor = HealthMonitor(self.registry)

    def register(self, module: BaseModule) -> None:
        self.registry.register(module)
        module.init()

    def health_all(self) -> Dict[str, Any]:
        return health_all(self.registry)

    def dispatch(self, topic: str, *args: Any, **kwargs: Any) -> None:
        self.events.publish(topic, *args, **kwargs)

    def status(self) -> Dict[str, Any]:
        health = self.health_all()
        report = self.health_monitor.report()
        return {
            "ok": health["ok"],
            "modules": list(self.registry.all_names()),
            "health": health,
            "module_states": report.get("module_states", health["modules"]),
            "latencies_ms": report.get("latencies_ms", {}),
        }


__all__ = ["Kernel", "BaseModule", "HealthCheckMixin", "ModuleRegistry", "EventBus", "HealthMonitor", "health_all"]
