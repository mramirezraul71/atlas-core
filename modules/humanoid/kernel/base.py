"""Base classes for humanoid modules: BaseModule, HealthCheckMixin."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict


class BaseModule(ABC):
    """Base for all humanoid submodules. Subclasses register with the kernel."""

    name: str = "base"
    enabled: bool = True

    @abstractmethod
    def init(self) -> None:
        """Initialize the module (load deps, connect). No heavy work."""
        ...

    def shutdown(self) -> None:
        """Clean shutdown. Override if needed."""
        pass

    def health(self) -> Dict[str, Any]:
        """Report module health. Default uses health_check() if present."""
        if hasattr(self, "health_check"):
            return self.health_check()
        return {"ok": True, "message": "ok", "details": {}}

    def info(self) -> Dict[str, Any]:
        """Metadata for status. Override to add fields."""
        return {"module": self.name, "enabled": self.enabled}


class HealthCheckMixin:
    """Mixin that adds health_check() returning ok, message, details."""

    def health_check(self) -> Dict[str, Any]:
        """Override to implement real check. Default: ok=True."""
        return {"ok": True, "message": "ok", "details": {}}
