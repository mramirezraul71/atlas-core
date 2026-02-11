"""Humanoid comms: telegram bridge."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .telegram_bridge import TelegramBridge


class CommsModule(BaseModule, HealthCheckMixin):
    name = "comms"

    def __init__(self) -> None:
        self.telegram = TelegramBridge()

    def init(self) -> None:
        pass

    def health_check(self) -> Dict[str, Any]:
        return self.telegram.health_check()

    def info(self) -> Dict[str, Any]:
        return {"module": self.name}


__all__ = ["CommsModule", "TelegramBridge"]
