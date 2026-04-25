from __future__ import annotations

from dataclasses import dataclass
from time import time
import os


@dataclass(frozen=True)
class CircuitBreakerSnapshot:
    name: str
    state: str
    consecutive_failures: int
    failure_threshold: int
    cooldown_sec: int
    opened_at: float | None
    last_error: str | None


class CircuitBreaker:
    def __init__(self, *, name: str, failure_threshold: int, cooldown_sec: int) -> None:
        self.name = name
        self.failure_threshold = max(1, int(failure_threshold))
        self.cooldown_sec = max(1, int(cooldown_sec))
        self._state = "closed"
        self._consecutive_failures = 0
        self._opened_at: float | None = None
        self._last_error: str | None = None

    def allow_request(self) -> bool:
        if self._state == "closed":
            return True
        now = time()
        if self._opened_at is None:
            self._state = "half_open"
            return True
        if (now - self._opened_at) >= self.cooldown_sec:
            self._state = "half_open"
            return True
        return False

    def record_success(self) -> None:
        self._consecutive_failures = 0
        self._state = "closed"
        self._opened_at = None
        self._last_error = None

    def record_failure(self, error: str) -> None:
        self._last_error = error
        self._consecutive_failures += 1
        if self._state == "half_open" or self._consecutive_failures >= self.failure_threshold:
            self._state = "open"
            self._opened_at = time()

    def snapshot(self) -> CircuitBreakerSnapshot:
        return CircuitBreakerSnapshot(
            name=self.name,
            state=self._state,
            consecutive_failures=self._consecutive_failures,
            failure_threshold=self.failure_threshold,
            cooldown_sec=self.cooldown_sec,
            opened_at=self._opened_at,
            last_error=self._last_error,
        )


_BREAKERS: dict[str, CircuitBreaker] = {}


def resolve_provider_circuit_breaker(name: str) -> CircuitBreaker:
    threshold = int(os.getenv("ATLAS_PROVIDER_CB_FAILURE_THRESHOLD", "3"))
    cooldown = int(os.getenv("ATLAS_PROVIDER_CB_COOLDOWN_SEC", "120"))
    breaker = _BREAKERS.get(name)
    if breaker is None:
        breaker = CircuitBreaker(name=name, failure_threshold=threshold, cooldown_sec=cooldown)
        _BREAKERS[name] = breaker
    return breaker


def reset_provider_circuit_breakers() -> None:
    _BREAKERS.clear()
