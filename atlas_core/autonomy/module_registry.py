from __future__ import annotations

from typing import Any, Dict, List, Protocol, runtime_checkable

from .models import Command, ModuleRisks, ModuleState


@runtime_checkable
class AutonomyModule(Protocol):
    name: str

    def get_capabilities(self) -> Dict[str, Any]:
        ...

    def get_state(self) -> ModuleState:
        ...

    def get_risks(self) -> ModuleRisks:
        ...

    def apply_command(self, command: Command) -> Dict[str, Any]:
        ...


class ModuleRegistry:
    def __init__(self) -> None:
        self._modules: Dict[str, AutonomyModule] = {}

    def register(self, module: AutonomyModule) -> None:
        self._modules[module.name] = module

    def get(self, name: str) -> AutonomyModule | None:
        return self._modules.get(name)

    def all(self) -> List[AutonomyModule]:
        return list(self._modules.values())

