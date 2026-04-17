from __future__ import annotations

import logging
import threading
import time
from typing import Iterable

from .models import Command
from .module_registry import AutonomyModule, ModuleRegistry
from .policy_engine import PolicyEngine
from .state_bus import StateBus

logger = logging.getLogger(__name__)


class AutonomyOrchestrator:
    def __init__(
        self,
        modules: Iterable[AutonomyModule],
        interval_seconds: int = 30,
        config: dict | None = None,
    ) -> None:
        self.registry = ModuleRegistry()
        for m in modules:
            self.registry.register(m)

        self.state_bus = StateBus(self.registry)
        self.policy_engine = PolicyEngine(self.registry, self.state_bus, config=config)
        self.interval_seconds = interval_seconds
        self._running = False
        self._thread: threading.Thread | None = None

    def _tick(self) -> None:
        try:
            commands = self.policy_engine.evaluate()
            self._apply_commands(commands)
        except Exception as exc:
            logger.exception("Error in AutonomyOrchestrator tick: %s", exc)

    def _apply_commands(self, commands: list[Command]) -> None:
        for cmd in commands:
            module = self.registry.get(cmd.target)
            if not module:
                logger.warning("Command for unknown module %s: %s", cmd.target, cmd)
                continue
            try:
                result = module.apply_command(cmd)
                logger.info("Applied command to %s: %s -> %s", cmd.target, cmd, result)
            except Exception as exc:
                logger.exception(
                    "Error applying command %s to module %s: %s",
                    cmd,
                    module.name,
                    exc,
                )

    def start_background(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("AutonomyOrchestrator started in background")

    def _run_loop(self) -> None:
        while self._running:
            self._tick()
            time.sleep(self.interval_seconds)

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        logger.info("AutonomyOrchestrator stopped")

    def run_once(self) -> None:
        self._tick()

