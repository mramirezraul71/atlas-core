from __future__ import annotations

from typing import Any, Dict

from .models import Snapshot
from .module_registry import ModuleRegistry


class StateBus:
    def __init__(self, registry: ModuleRegistry) -> None:
        self._registry = registry
        self._last_snapshot: Snapshot | None = None

    def collect_snapshot(self) -> Snapshot:
        modules: Dict[str, Dict[str, Any]] = {}
        for module in self._registry.all():
            state = module.get_state()
            risks = module.get_risks()
            caps = module.get_capabilities()
            modules[module.name] = {
                "state": state,
                "risks": risks,
                "capabilities": caps,
            }
        snapshot = Snapshot(modules=modules)
        self._last_snapshot = snapshot
        return snapshot

    @property
    def last_snapshot(self) -> Snapshot | None:
        return self._last_snapshot

