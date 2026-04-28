"""
lotto_quant.watchdog.lotto_watchdog
===================================

Heartbeat + circuit breaker for the radar loop.

The watchdog wraps a callable (typically `LottoQuantRadar.run_scan_cycle`)
and:
    - records consecutive failures
    - opens the circuit after N failures
    - keeps it open for `cooldown_s` before retrying
    - exposes a sync `is_open()` state for the dashboard
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Awaitable, Callable, Optional

from .. import config

logger = logging.getLogger(__name__)


class CircuitBreaker:
    """Simple circuit breaker."""

    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"

    def __init__(
        self,
        max_failures: int = config.WATCHDOG_MAX_CONSECUTIVE_FAILURES,
        cooldown_s: int = config.CIRCUIT_BREAKER_COOLDOWN_S,
    ):
        self.max_failures = max_failures
        self.cooldown_s = cooldown_s
        self._failures = 0
        self._state = self.CLOSED
        self._opened_at: Optional[float] = None

    def record_success(self) -> None:
        if self._state == self.HALF_OPEN:
            logger.info("Circuit breaker recovered → CLOSED")
        self._failures = 0
        self._state = self.CLOSED
        self._opened_at = None

    def record_failure(self) -> None:
        self._failures += 1
        if self._failures >= self.max_failures and self._state != self.OPEN:
            self._state = self.OPEN
            self._opened_at = time.time()
            logger.error(
                "Circuit breaker OPENED after %d consecutive failures (cooldown=%ds)",
                self._failures, self.cooldown_s,
            )

    def is_open(self) -> bool:
        if self._state != self.OPEN:
            return False
        if self._opened_at is None:
            return True
        if (time.time() - self._opened_at) >= self.cooldown_s:
            self._state = self.HALF_OPEN
            logger.info("Circuit breaker → HALF_OPEN (probing)")
            return False
        return True

    @property
    def state(self) -> str:
        return self._state

    @property
    def failures(self) -> int:
        return self._failures


class LottoWatchdog:
    """
    Wraps an async cycle function with heartbeat logging + circuit breaker.

    Usage
    -----
        wd = LottoWatchdog()
        async def cycle():
            return await radar.run_scan_cycle()
        await wd.run_forever(cycle, interval_s=3600)
    """

    def __init__(
        self,
        heartbeat_s: int = config.WATCHDOG_HEARTBEAT_S,
        breaker: Optional[CircuitBreaker] = None,
    ):
        self.heartbeat_s = heartbeat_s
        self.breaker = breaker or CircuitBreaker()
        self._running = False
        self.last_success_ts: Optional[float] = None
        self.last_failure_ts: Optional[float] = None
        self._cycles = 0

    async def run_forever(
        self,
        cycle_fn: Callable[[], Awaitable[Any]],
        interval_s: int = 3600,
    ) -> None:
        self._running = True
        while self._running:
            if self.breaker.is_open():
                logger.warning(
                    "Circuit breaker OPEN — sleeping %ds before next probe",
                    self.heartbeat_s,
                )
                await asyncio.sleep(self.heartbeat_s)
                continue
            try:
                await cycle_fn()
                self._cycles += 1
                self.last_success_ts = time.time()
                self.breaker.record_success()
                logger.info("Watchdog heartbeat OK (cycle #%d)", self._cycles)
            except Exception as e:
                self.last_failure_ts = time.time()
                self.breaker.record_failure()
                logger.exception("Cycle failed (consecutive failures=%d): %s",
                                 self.breaker.failures, e)
            await asyncio.sleep(interval_s)

    def stop(self) -> None:
        self._running = False
