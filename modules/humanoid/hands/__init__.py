"""Humanoid hands: safe_shell, fs_controller, process_manager, interpreter_bridge."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .safe_shell import SafeShellExecutor
from .fs_controller import FileSystemController
from .process_manager import ProcessManager


def _get_interpreter_bridge():
    """Lazy import to avoid hard dependency on open-interpreter at startup."""
    try:
        from .interpreter_bridge import get_session_manager, interpreter_status
        return get_session_manager, interpreter_status
    except ImportError:
        return None, None


class HandsModule(BaseModule, HealthCheckMixin):
    name = "hands"

    def __init__(self) -> None:
        self.shell = SafeShellExecutor()
        self.fs = FileSystemController()
        self.process = ProcessManager()
        self._interpreter_mgr = None

    @property
    def interpreter(self):
        """Lazy-loaded InterpreterSessionManager."""
        if self._interpreter_mgr is None:
            get_mgr, _ = _get_interpreter_bridge()
            if get_mgr:
                self._interpreter_mgr = get_mgr()
        return self._interpreter_mgr

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        _, status_fn = _get_interpreter_bridge()
        interp_ok = False
        if status_fn:
            try:
                interp_ok = status_fn().get("installed", False)
            except Exception:
                pass
        return {
            "ok": True,
            "message": "ok",
            "details": {"shell": True, "fs": True, "process": True, "interpreter": interp_ok},
        }

    def info(self) -> Dict[str, Any]:
        return {
            "module": self.name,
            "components": ["SafeShellExecutor", "FileSystemController", "ProcessManager", "InterpreterBridge"],
        }


__all__ = ["HandsModule", "SafeShellExecutor", "FileSystemController", "ProcessManager"]
