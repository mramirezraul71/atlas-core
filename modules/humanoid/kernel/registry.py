"""Dynamic registry of humanoid modules."""
from __future__ import annotations

from typing import Dict, List, Optional

from .base import BaseModule


class ModuleRegistry:
    """Kernel uses this to hold and lookup modules."""

    def __init__(self) -> None:
        self._modules: Dict[str, BaseModule] = {}

    def register(self, module: BaseModule) -> None:
        self._modules[module.name] = module

    def get(self, name: str) -> Optional[BaseModule]:
        return self._modules.get(name)

    def all_names(self) -> List[str]:
        return list(self._modules.keys())

    def all_modules(self) -> Dict[str, BaseModule]:
        return dict(self._modules)
