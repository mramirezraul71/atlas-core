"""Bus de estado global para el Brain Core."""
from __future__ import annotations

import logging
import threading
import time
from copy import deepcopy
from typing import Any

from .models import ModuleState, RiskState, SystemSnapshot

logger = logging.getLogger("atlas.brain.state_bus")


class StateBus:
    """Estado coherente: módulos, riesgos, modo global, misión, resultados."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._global_mode: str = "semi"
        self._active_mission: str | None = None
        self._modules: dict[str, ModuleState] = {}
        self._risks: dict[str, RiskState] = {}
        self._execution_log: list[dict[str, Any]] = []
        self._meta: dict[str, Any] = {}

    def update_module_state(self, module_name: str, state: ModuleState) -> None:
        with self._lock:
            self._modules[module_name] = state
            logger.debug("module_state %s health=%s", module_name, state.health)

    def update_risk(self, module_name: str, risk: RiskState) -> None:
        with self._lock:
            self._risks[module_name] = risk
            logger.debug("risk %s level=%s", module_name, risk.level)

    def set_global_mode(self, mode: str) -> None:
        with self._lock:
            self._global_mode = mode
            logger.info("global_mode=%s", mode)

    def set_active_mission(self, mission_name: str | None) -> None:
        with self._lock:
            self._active_mission = mission_name
            logger.debug("active_mission=%s", mission_name)

    def record_execution_result(self, module_name: str, result: dict[str, Any]) -> None:
        with self._lock:
            entry = {"module": module_name, "result": deepcopy(result)}
            self._execution_log.append(entry)
            if len(self._execution_log) > 500:
                self._execution_log = self._execution_log[-250:]
            logger.debug("execution_result %s ok=%s", module_name, result.get("ok"))

    def get_snapshot(self) -> SystemSnapshot:
        with self._lock:
            return SystemSnapshot(
                global_mode=self._global_mode,
                active_mission=self._active_mission,
                modules=deepcopy(self._modules),
                risks=deepcopy(self._risks),
                meta={
                    **deepcopy(self._meta),
                    "execution_count": len(self._execution_log),
                },
            )

    def get_execution_tail(self, n: int = 10) -> list[dict[str, Any]]:
        with self._lock:
            return deepcopy(self._execution_log[-n:])

    def append_fail_safe(self, reason: str) -> None:
        with self._lock:
            hist = self._meta.setdefault("fail_safe_history", [])
            hist.append(reason)
            if len(hist) > 50:
                del hist[:-25]

    def set_meta(self, key: str, value: Any) -> None:
        with self._lock:
            self._meta[key] = deepcopy(value)

    def increment_meta_counter(self, key: str, delta: int = 1) -> int:
        with self._lock:
            cur = int(self._meta.get(key, 0))
            cur += int(delta)
            self._meta[key] = cur
            return cur

    def append_meta_sample(self, key: str, value: Any, max_items: int = 200) -> None:
        with self._lock:
            arr = self._meta.setdefault(key, [])
            if not isinstance(arr, list):
                arr = []
            arr.append(deepcopy(value))
            if len(arr) > max(10, max_items):
                arr = arr[-max(10, max_items // 2) :]
            self._meta[key] = arr

    def mark_loop_start(self, loop_name: str) -> float:
        started = time.monotonic()
        self.set_meta(f"{loop_name}_started_monotonic", started)
        return started

    def mark_loop_end(self, loop_name: str, started: float) -> float:
        elapsed = max(0.0, time.monotonic() - started)
        self.increment_meta_counter(f"{loop_name}_cycles", 1)
        self.set_meta(f"{loop_name}_last_elapsed_sec", elapsed)
        self.append_meta_sample(f"{loop_name}_elapsed_samples", elapsed, max_items=300)
        return elapsed
