"""Circuit Breaker — F6 (esqueleto).

Protección frente a cascadas de errores y secuencias de pérdidas.

Estados:
- ``closed``    : flujo normal.
- ``open``      : bloquea órdenes durante ``cooldown_seconds`` tras un trigger.
- ``half_open`` : ventana exploratoria — admite 1 orden de prueba.

Triggers configurables:
- ``max_consecutive_losses`` (default 3)
- ``max_failed_orders``      (default 5)
- ``cooldown_seconds``       (default 300)
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Literal


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


CircuitState = Literal["closed", "open", "half_open"]


@dataclass(slots=True)
class CircuitConfig:
    max_consecutive_losses: int = 3
    max_failed_orders: int = 5
    cooldown_seconds: int = 300

    @classmethod
    def from_env(cls) -> "CircuitConfig":
        return cls(
            max_consecutive_losses=_env_int("ATLAS_CB_MAX_CONSEC_LOSSES", 3),
            max_failed_orders=_env_int("ATLAS_CB_MAX_FAILED_ORDERS", 5),
            cooldown_seconds=_env_int("ATLAS_CB_COOLDOWN_SECONDS", 300),
        )


@dataclass(slots=True)
class CircuitBreaker:
    config: CircuitConfig = field(default_factory=CircuitConfig.from_env)
    state: CircuitState = "closed"
    consecutive_losses: int = 0
    failed_orders: int = 0
    opened_at: float = 0.0

    def _now(self) -> float:
        return time.time()

    # ── entrada de eventos ────────────────────────────────────────────────
    def record_loss(self) -> None:
        self.consecutive_losses += 1
        if self.consecutive_losses >= self.config.max_consecutive_losses:
            self._trip("max_consecutive_losses")

    def record_win(self) -> None:
        self.consecutive_losses = 0

    def record_order_failure(self) -> None:
        self.failed_orders += 1
        if self.failed_orders >= self.config.max_failed_orders:
            self._trip("max_failed_orders")

    def reset_failures(self) -> None:
        self.failed_orders = 0

    # ── transición ────────────────────────────────────────────────────────
    def _trip(self, _reason: str) -> None:
        self.state = "open"
        self.opened_at = self._now()

    def can_submit(self) -> bool:
        if self.state == "closed":
            return True
        if self.state == "open":
            if self._now() - self.opened_at >= self.config.cooldown_seconds:
                self.state = "half_open"
                return True
            return False
        # half_open: admite una y volverá a closed en confirm_recovery()
        return True

    def confirm_recovery(self) -> None:
        if self.state == "half_open":
            self.state = "closed"
            self.consecutive_losses = 0
            self.failed_orders = 0
