"""
SurvivalMode - Modo de operación mínima viable cuando recursos o fallos son críticos.
Desactiva tareas no esenciales; mantiene health checks y API core.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_def_config = {
    "auto_trigger": True,
    "triggers": {"cpu_critical": 95, "ram_critical": 95, "error_rate_critical": 50},
}


class SurvivalMode:
    _active = False
    _reason = ""

    def __init__(self, config: dict | None = None):
        self._config = config or _def_config

    def enter_survival_mode(self, reason: str = "") -> None:
        """Activa modo supervivencia: logging mínimo, solo funciones core."""
        SurvivalMode._active = True
        SurvivalMode._reason = reason or "manual"
        logger.warning("Survival mode ENTERED: %s", reason)

    def exit_survival_mode(self) -> None:
        """Restaura operación normal."""
        SurvivalMode._active = False
        SurvivalMode._reason = ""
        logger.info("Survival mode EXITED")

    @classmethod
    def is_in_survival(cls) -> bool:
        return cls._active

    def get_disabled_features(self) -> list[str]:
        """Lista de características desactivadas en survival."""
        if not SurvivalMode._active:
            return []
        return [
            "verbose_logging",
            "background_tasks_non_essential",
            "telemetry_detailed",
            "evolution_checks",
            "analytics",
        ]
