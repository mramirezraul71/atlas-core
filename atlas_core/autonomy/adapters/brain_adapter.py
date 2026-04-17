from __future__ import annotations

from typing import Any, Dict

from ..models import Command, ModuleRisks, ModuleState
from ..module_registry import AutonomyModule


class BrainAutonomyAdapter(AutonomyModule):
    name = "brain"

    def get_capabilities(self) -> Dict[str, Any]:
        return {"type": "brain", "models": ["gpt", "deepseek", "qwen"]}

    def get_state(self) -> ModuleState:
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

