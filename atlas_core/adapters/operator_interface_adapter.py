"""Canal operador — mensajes y feedback, sin física."""
from __future__ import annotations

import logging
from typing import Any

from ..brain.models import Command, ModuleState, RiskState

logger = logging.getLogger("atlas.brain.adapter.operator")


class OperatorInterfaceAdapter:
    name = "operator"

    def __init__(self) -> None:
        self._inbox: list[dict[str, Any]] = []

    def get_state(self) -> ModuleState:
        return ModuleState(name=self.name, health="ok", mode="ready", details={"pending": len(self._inbox)})

    def get_risks(self) -> RiskState:
        return RiskState(name=self.name, level="low", details={})

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        if a == "notify":
            msg = {"type": "notify", "payload": command.params}
            self._inbox.append(msg)
            logger.info("operator notify: %s", str(command.params)[:120])
            return {"ok": True, "queued": True}
        if a == "confirm":
            return {"ok": True, "confirmed": True, "params": command.params}
        if a == "reject":
            return {"ok": True, "rejected": True, "params": command.params}
        return {"ok": False, "error": f"unsupported:{a}"}
