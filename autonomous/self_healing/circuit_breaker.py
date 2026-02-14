"""
CircuitBreaker - Patrón circuit breaker: CLOSED → OPEN → HALF_OPEN.
Evita cascadas de fallos; configurable por threshold y timeout.
"""
from __future__ import annotations

import functools
import logging
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Any, Callable, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")


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


class CircuitState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"


class CircuitBreakerError(Exception):
    """Se lanza cuando el circuito está OPEN y no se permite la llamada."""

    def __init__(self, name: str, state: CircuitState):
        self.name = name
        self.state = state
        super().__init__(f"CircuitBreaker '{name}' is {state.value}")


class CircuitBreaker:
    """
    Circuit breaker por nombre (por servicio/función).
    failure_threshold: N fallos para abrir.
    timeout_seconds: tiempo en OPEN antes de pasar a HALF_OPEN.
    success_threshold: éxitos en HALF_OPEN para cerrar.
    """

    _instances: dict[str, "CircuitBreaker"] = {}
    _lock = threading.Lock()

    def __init__(
        self,
        name: str = "default",
        failure_threshold: int = 5,
        timeout_seconds: float = 60,
        success_threshold: int = 2,
    ):
        self.name = name
        self.failure_threshold = failure_threshold
        self.timeout_seconds = timeout_seconds
        self.success_threshold = success_threshold
        self._state = CircuitState.CLOSED
        self._failures = 0
        self._successes = 0
        self._last_failure_time: float = 0
        self._lock = threading.Lock()

    @classmethod
    def get(cls, name: str = "default", config: dict | None = None) -> "CircuitBreaker":
        """Obtiene o crea instancia por nombre."""
        cfg = config or _load_config().get("self_healing", {}).get("circuit_breaker", {})
        with cls._lock:
            if name not in cls._instances:
                cls._instances[name] = CircuitBreaker(
                    name=name,
                    failure_threshold=int(cfg.get("failure_threshold", 5)),
                    timeout_seconds=float(cfg.get("timeout_seconds", 60)),
                    success_threshold=int(cfg.get("success_threshold", 2)),
                )
            return cls._instances[name]

    def get_state(self) -> CircuitState:
        with self._lock:
            if self._state == CircuitState.OPEN:
                if time.time() - self._last_failure_time >= self.timeout_seconds:
                    self._state = CircuitState.HALF_OPEN
                    self._successes = 0
            return self._state

    def call(self, func: Callable[..., T], *args: Any, **kwargs: Any) -> T:
        """Ejecuta func si el circuito lo permite; si no, lanza CircuitBreakerError."""
        state = self.get_state()
        if state == CircuitState.OPEN:
            raise CircuitBreakerError(self.name, state)
        try:
            result = func(*args, **kwargs)
            self.record_success()
            return result
        except Exception as e:
            self.record_failure()
            raise

    def record_success(self) -> None:
        with self._lock:
            if self._state == CircuitState.HALF_OPEN:
                self._successes += 1
                if self._successes >= self.success_threshold:
                    self._state = CircuitState.CLOSED
                    self._failures = 0
            elif self._state == CircuitState.CLOSED:
                self._failures = 0

    def record_failure(self) -> None:
        with self._lock:
            self._last_failure_time = time.time()
            if self._state == CircuitState.HALF_OPEN:
                self._state = CircuitState.OPEN
                self._failures = self.failure_threshold
                return
            self._failures += 1
            if self._failures >= self.failure_threshold:
                self._state = CircuitState.OPEN

    def reset(self) -> None:
        """Fuerza estado CLOSED."""
        with self._lock:
            self._state = CircuitState.CLOSED
            self._failures = 0
            self._successes = 0


def circuit_breaker(name: str = "default"):
    """Decorador para aplicar circuit breaker a una función."""

    def decorator(f: Callable[..., T]) -> Callable[..., T]:
        @functools.wraps(f)
        def wrapper(*args: Any, **kwargs: Any) -> T:
            cb = CircuitBreaker.get(name)
            return cb.call(f, *args, **kwargs)
        return wrapper
    return decorator
