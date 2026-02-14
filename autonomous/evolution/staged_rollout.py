"""
StagedRollout - Despliegue gradual: CANARY → BETA → STABLE → FULL.
En single-instance: fases por tipo de componente (deps, tools, brain, core).
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class RolloutPhase(str, Enum):
    CANARY = "canary"
    BETA = "beta"
    STABLE = "stable"
    FULL = "full"


@dataclass
class RolloutStatus:
    phase: RolloutPhase
    started_at: float
    phase_duration_min: float
    metrics_ok: bool
    message: str


class StagedRollout:
    """
    En contexto ATLAS (una instancia), las fases se traducen a:
    CANARY: actualizar solo dependencias no críticas
    BETA: módulos de tools
    STABLE: brain components
    FULL: core systems
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("evolution", {})
        self._rollout_cfg = self._config.get("staged_rollout", {})
        self._phase_duration_min = float(self._rollout_cfg.get("phase_duration_minutes", 10))
        self._current_phase: RolloutPhase | None = None
        self._phase_started_at: float = 0
        self._metrics_check_callback: Callable[[], bool] | None = None

    def start_rollout(self, update_package: dict[str, Any] | None = None) -> RolloutStatus:
        """Inicia rollout en fase CANARY."""
        self._current_phase = RolloutPhase.CANARY
        self._phase_started_at = time.time()
        logger.info("Staged rollout started: CANARY")
        return self.get_rollout_status()

    def advance_phase(self) -> bool:
        """Avanza a la siguiente fase si métricas OK. Retorna True si avanzó."""
        if self._current_phase is None:
            return False
        if self._metrics_check_callback and not self._metrics_check_callback():
            return False
        order = [RolloutPhase.CANARY, RolloutPhase.BETA, RolloutPhase.STABLE, RolloutPhase.FULL]
        idx = order.index(self._current_phase)
        if idx >= len(order) - 1:
            self._current_phase = None
            logger.info("Staged rollout completed: FULL")
            return True
        self._current_phase = order[idx + 1]
        self._phase_started_at = time.time()
        logger.info("Staged rollout advanced to: %s", self._current_phase.value)
        return True

    def rollback_phase(self) -> bool:
        """Vuelve a la fase anterior (o cancela si estaba en CANARY)."""
        if self._current_phase is None:
            return True
        order = [RolloutPhase.CANARY, RolloutPhase.BETA, RolloutPhase.STABLE, RolloutPhase.FULL]
        idx = order.index(self._current_phase)
        if idx <= 0:
            self._current_phase = None
            logger.warning("Staged rollout rolled back: cancelled")
            return True
        self._current_phase = order[idx - 1]
        self._phase_started_at = time.time()
        logger.warning("Staged rollout rolled back to: %s", self._current_phase.value)
        return True

    def get_rollout_status(self) -> RolloutStatus:
        """Estado actual del rollout."""
        metrics_ok = True
        if self._metrics_check_callback:
            try:
                metrics_ok = self._metrics_check_callback()
            except Exception:
                metrics_ok = False
        return RolloutStatus(
            phase=self._current_phase or RolloutPhase.FULL,
            started_at=self._phase_started_at,
            phase_duration_min=self._phase_duration_min,
            metrics_ok=metrics_ok,
            message=f"Phase {self._current_phase.value if self._current_phase else 'none'}",
        )

    def set_metrics_check(self, callback: Callable[[], bool]) -> None:
        """Registra callback para validar métricas antes de advance_phase."""
        self._metrics_check_callback = callback
