"""Visión — stub con confianza simulada."""
from __future__ import annotations

import logging
from typing import Any

from ..brain.models import Command, ModuleState, RiskState

logger = logging.getLogger("atlas.brain.adapter.vision")


class VisionBrainAdapter:
    name = "vision"

    def __init__(self) -> None:
        self._mode = "observe"
        self._confidence = 0.85

    def get_state(self) -> ModuleState:
        return ModuleState(
            name=self.name,
            health="ok",
            mode=self._mode,
            details={"confidence": self._confidence, "stub": True},
        )

    def get_risks(self) -> RiskState:
        lvl: Any = "low" if self._confidence >= 0.5 else "medium"
        return RiskState(name=self.name, level=lvl, details={"confidence": self._confidence})

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        if a == "set_mode":
            self._mode = str(command.params.get("mode", "observe"))
            return {"ok": True, "mode": self._mode}
        if a == "observe":
            logger.debug("vision observe (stub)")
            return {"ok": True, "confidence": self._confidence, "stub": True}
        return {"ok": False, "error": f"unsupported:{a}"}
