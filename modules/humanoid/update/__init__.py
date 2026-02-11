"""Humanoid update: env_scanner, dep_resolver, updater."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .env_scanner import EnvScanner
from .dep_resolver import DepResolver
from .updater import Updater


class UpdateModule(BaseModule, HealthCheckMixin):
    name = "update"

    def __init__(self) -> None:
        self.scanner = EnvScanner()
        self.resolver = DepResolver()
        self.updater = Updater()

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        pv = self.scanner.python_version()
        return {
            "ok": pv.get("ok", False),
            "message": "ok" if pv.get("ok") else pv.get("error"),
            "details": {"python": pv.get("version")},
        }

    def info(self) -> Dict[str, Any]:
        return {"module": self.name}


__all__ = ["UpdateModule", "EnvScanner", "DepResolver", "Updater"]
