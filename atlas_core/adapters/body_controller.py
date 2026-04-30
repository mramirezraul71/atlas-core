"""Control de cuerpo — stub seguro, sin hardware real."""
from __future__ import annotations

import logging
from typing import Any

from ..brain.models import Command, ModuleState, RiskState

logger = logging.getLogger("atlas.brain.adapter.body")


class BodyControllerAdapter:
    name = "body"

    def __init__(self) -> None:
        self._mode = "idle"
        self._safe = True
        self._last_speak: str | None = None

    def get_state(self) -> ModuleState:
        health: Any = "ok" if self._safe else "degraded"
        return ModuleState(
            name=self.name,
            health=health,
            mode=self._mode,
            details={"stub": True, "last_speak": self._last_speak},
        )

    def get_risks(self) -> RiskState:
        return RiskState(name=self.name, level="low", details={})

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        if a == "speak":
            self._last_speak = str(command.params.get("text", "") or command.params.get("message", ""))
            logger.info("body speak (stub): %s", self._last_speak[:80])
            return {"ok": True, "action": "speak", "stub": True}
        if a == "set_mode":
            self._mode = str(command.params.get("mode", "idle"))
            return {"ok": True, "mode": self._mode}
        if a == "stop":
            self._mode = "stopped"
            return {"ok": True, "stopped": True}
        return {"ok": False, "error": f"unsupported_action:{command.action}"}
