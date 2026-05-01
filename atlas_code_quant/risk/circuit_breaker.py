"""Atlas Code Quant — Circuit breaker global (F19).

Circuit breaker minimalista con estados ``closed``, ``open`` y
``half_open``, contador de violaciones consecutivas y ventana
temporal de cooldown.

Reglas duras F19:

* Sólo afecta al pipeline paper. NO autoriza ni bloquea órdenes
  reales.
* Defensivo: ningún método lanza.
* Reloj inyectable (``time_fn``) para tests deterministas.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

logger = logging.getLogger("atlas.code_quant.risk.circuit_breaker")


__all__ = [
    "BreakerState",
    "CircuitBreakerConfig",
    "CircuitBreaker",
]


class BreakerState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"


@dataclass(frozen=True)
class CircuitBreakerConfig:
    failure_threshold: int = 3  # violaciones consecutivas antes de abrir
    cooldown_seconds: float = 60.0  # tiempo en open antes de half_open


class CircuitBreaker:
    """Breaker global de Atlas paper (F19)."""

    def __init__(
        self,
        *,
        config: CircuitBreakerConfig | None = None,
        time_fn: Callable[[], float] | None = None,
    ) -> None:
        self._cfg = config or CircuitBreakerConfig()
        self._time = time_fn or time.monotonic
        self._state: BreakerState = BreakerState.CLOSED
        self._consecutive_failures: int = 0
        self._opened_at: float | None = None

    # ---------------------------------------------------------------- props

    @property
    def state(self) -> BreakerState:
        # transición lazy de OPEN→HALF_OPEN al expirar cooldown
        if self._state == BreakerState.OPEN and self._cooldown_elapsed():
            prev = self._state
            self._state = BreakerState.HALF_OPEN
            logger.info(
                "safety_event %s",
                {
                    "type": "safety_event",
                    "safety_type": "circuit_breaker",
                    "triggered_by": "cooldown_elapsed",
                    "value": self._consecutive_failures,
                    "threshold": self._cfg.failure_threshold,
                    "state": None,
                    "trace_id": None,
                    "action": "half_open",
                    "reason": f"{prev.value}_to_half_open",
                },
            )
        return self._state

    @property
    def consecutive_failures(self) -> int:
        return self._consecutive_failures

    @property
    def config(self) -> CircuitBreakerConfig:
        return self._cfg

    # ------------------------------------------------------------ commands

    def record_success(self) -> None:
        """Una operación OK. Si estábamos en half_open, reset a closed."""
        try:
            prev = self._state
            self._consecutive_failures = 0
            if self._state in (BreakerState.HALF_OPEN, BreakerState.OPEN):
                logger.info(
                    "circuit_breaker: success while %s → CLOSED", self._state.value
                )
            self._state = BreakerState.CLOSED
            self._opened_at = None
            if prev in (BreakerState.OPEN, BreakerState.HALF_OPEN):
                logger.info(
                    "safety_event %s",
                    {
                        "type": "safety_event",
                        "safety_type": "circuit_breaker",
                        "triggered_by": "success",
                        "value": 0,
                        "threshold": self._cfg.failure_threshold,
                        "state": None,
                        "trace_id": None,
                        "action": "close_breaker",
                        "reason": f"{prev.value}_to_closed",
                    },
                )
        except Exception as exc:  # noqa: BLE001
            logger.warning("circuit_breaker.record_success raised: %s", exc)

    def record_failure(self) -> BreakerState:
        """Una violación. Devuelve el estado tras procesarla."""
        try:
            prev = self._state
            self._consecutive_failures += 1
            if self._state == BreakerState.HALF_OPEN:
                # cualquier fallo en half_open re-abre inmediatamente
                self._state = BreakerState.OPEN
                self._opened_at = self._time()
                logger.warning(
                    "safety_event %s",
                    {
                        "type": "safety_event",
                        "safety_type": "circuit_breaker",
                        "triggered_by": "consecutive_failures",
                        "value": self._consecutive_failures,
                        "threshold": self._cfg.failure_threshold,
                        "state": None,
                        "trace_id": None,
                        "action": "open_breaker",
                        "reason": f"{prev.value}_failure_reopen",
                    },
                )
                return self._state
            if self._consecutive_failures >= self._cfg.failure_threshold:
                self._state = BreakerState.OPEN
                self._opened_at = self._time()
            if prev != self._state and self._state == BreakerState.OPEN:
                logger.warning(
                    "safety_event %s",
                    {
                        "type": "safety_event",
                        "safety_type": "circuit_breaker",
                        "triggered_by": "consecutive_failures",
                        "value": self._consecutive_failures,
                        "threshold": self._cfg.failure_threshold,
                        "state": None,
                        "trace_id": None,
                        "action": "open_breaker",
                        "reason": f"{prev.value}_to_open",
                    },
                )
            return self._state
        except Exception as exc:  # noqa: BLE001
            logger.warning("circuit_breaker.record_failure raised: %s", exc)
            return self._state

    def trip(self, reason: str = "manual_trip") -> None:
        """Abre el breaker manualmente."""
        logger.warning("circuit_breaker: tripped (%s)", reason)
        self._state = BreakerState.OPEN
        self._opened_at = self._time()
        self._consecutive_failures = max(
            self._consecutive_failures, self._cfg.failure_threshold
        )

    def reset(self) -> None:
        """Vuelve a CLOSED limpiando contadores."""
        self._state = BreakerState.CLOSED
        self._consecutive_failures = 0
        self._opened_at = None

    # -------------------------------------------------------------- helpers

    def allow_request(self) -> bool:
        """¿Puede el pipeline avanzar?

        * CLOSED → True.
        * HALF_OPEN → True (probe).
        * OPEN → True si ha expirado el cooldown (lazy → HALF_OPEN).
        """
        s = self.state  # propiedad transiciona lazy si toca
        return s in (BreakerState.CLOSED, BreakerState.HALF_OPEN)

    def _cooldown_elapsed(self) -> bool:
        if self._opened_at is None:
            return True
        return (self._time() - self._opened_at) >= self._cfg.cooldown_seconds
