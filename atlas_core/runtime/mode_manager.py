"""Cambios de modo global encapsulados."""
from __future__ import annotations

import logging

from ..brain.state_bus import StateBus

logger = logging.getLogger("atlas.brain.mode_manager")


class ModeManager:
    def __init__(self, state_bus: StateBus) -> None:
        self._bus = state_bus

    def set_mode(self, mode: str) -> None:
        logger.info("ModeManager -> global_mode=%s", mode)
        self._bus.set_global_mode(mode)
