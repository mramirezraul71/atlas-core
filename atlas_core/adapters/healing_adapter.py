"""Healing — autodiagnóstico stub."""
from __future__ import annotations

import logging
from typing import Any

from ..brain.models import Command, ModuleState, RiskState

logger = logging.getLogger("atlas.brain.adapter.healing")


class HealingBrainAdapter:
    name = "healing"

    def __init__(self) -> None:
        self._last_diag: str | None = None

    def get_state(self) -> ModuleState:
        return ModuleState(name=self.name, health="ok", mode="idle", details={"stub": True})

    def get_risks(self) -> RiskState:
        return RiskState(name=self.name, level="low", details={})

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        if a == "diagnose":
            self._last_diag = "no_issues_stub"
            return {"ok": True, "diagnosis": self._last_diag}
        if a == "recover":
            logger.info("healing recover (stub)")
            return {"ok": True, "recovered": True, "stub": True}
        return {"ok": False, "error": f"unsupported:{a}"}
