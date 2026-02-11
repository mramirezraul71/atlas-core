"""Humanoid hands: safe_shell, fs_controller, process_manager."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .safe_shell import SafeShellExecutor
from .fs_controller import FileSystemController
from .process_manager import ProcessManager


class HandsModule(BaseModule, HealthCheckMixin):
    name = "hands"

    def __init__(self) -> None:
        self.shell = SafeShellExecutor()
        self.fs = FileSystemController()
        self.process = ProcessManager()

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        return {"ok": True, "message": "ok", "details": {"shell": True, "fs": True, "process": True}}

    def info(self) -> Dict[str, Any]:
        return {"module": self.name, "components": ["SafeShellExecutor", "FileSystemController", "ProcessManager"]}


__all__ = ["HandsModule", "SafeShellExecutor", "FileSystemController", "ProcessManager"]
