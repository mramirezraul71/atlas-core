from __future__ import annotations

from typing import Any, Dict

from ..models import AutonomyMode, Command, ModuleRisks, ModuleState
from ..module_registry import AutonomyModule


class HealingAutonomyAdapter(AutonomyModule):
    name = "healing"

    def get_capabilities(self) -> Dict[str, Any]:
        return {"type": "healing", "supports_self_repair": True}

    def get_state(self) -> ModuleState:
        # Stub: still marked semi to be compatible with PolicyEngine assumptions.
        return ModuleState(
            name=self.name,
            mode="semi",  # type: ignore[arg-type]
            health="ok",
            details={},
        )

    def get_risks(self) -> ModuleRisks:
        return ModuleRisks(name=self.name, risks={})

    def apply_command(self, command: Command) -> Dict[str, Any]:
        return {"ok": True, "ignored": True, "command": {"action": command.action}}

