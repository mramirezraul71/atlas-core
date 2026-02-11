"""Health monitor: run health on all registered modules with latencies."""
from __future__ import annotations

import time
from typing import Any, Dict

from .base import BaseModule
from .registry import ModuleRegistry


def health_all(registry: ModuleRegistry) -> Dict[str, Any]:
    """Run health() on every module. Returns {ok, modules: {name: {..., ms}}, latencies}."""
    result: Dict[str, Any] = {"ok": True, "modules": {}, "latencies_ms": {}}
    for name, mod in registry.all_modules().items():
        t0 = time.perf_counter()
        try:
            h = mod.health() if hasattr(mod, "health") else mod.health_check() if hasattr(mod, "health_check") else {"ok": True, "message": "no health"}
            result["modules"][name] = dict(h) if isinstance(h, dict) else h
            result["latencies_ms"][name] = round((time.perf_counter() - t0) * 1000)
            if not result["modules"][name].get("ok", True):
                result["ok"] = False
        except Exception as e:
            result["modules"][name] = {"ok": False, "message": str(e)}
            result["latencies_ms"][name] = round((time.perf_counter() - t0) * 1000)
            result["ok"] = False
    return result


class HealthMonitor:
    """Reports module states and latencies. Uses registry to run health_all."""

    def __init__(self, registry: ModuleRegistry) -> None:
        self._registry = registry

    def report(self) -> Dict[str, Any]:
        """Returns {ok, module_states: {name: result}, latencies_ms}."""
        raw = health_all(self._registry)
        return {
            "ok": raw["ok"],
            "module_states": raw["modules"],
            "latencies_ms": raw.get("latencies_ms", {}),
        }
