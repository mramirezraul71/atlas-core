"""Enruta comandos al adapter registrado por target."""
from __future__ import annotations

import logging
from typing import Any

from .models import Command

logger = logging.getLogger("atlas.brain.command_router")


class CommandRouter:
    def __init__(self) -> None:
        self._adapters: dict[str, Any] = {}

    def register_adapter(self, name: str, adapter: Any) -> None:
        self._adapters[name] = adapter
        logger.debug("adapter registered: %s", name)

    def route(self, command: Command) -> dict[str, Any]:
        ad = self._adapters.get(command.target)
        if ad is None:
            return {"ok": False, "error": "unknown_target"}
        try:
            return ad.apply_command(command)
        except Exception as exc:
            logger.exception("route error target=%s", command.target)
            return {"ok": False, "error": str(exc)}
