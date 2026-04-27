"""Tests F6 — CircuitBreaker."""
from __future__ import annotations

import time

from atlas_code_quant.risk.circuit_breaker import CircuitBreaker, CircuitConfig


def test_starts_closed_and_allows_submit() -> None:
    cb = CircuitBreaker()
    assert cb.state == "closed"
    assert cb.can_submit() is True


def test_consecutive_losses_trip_open() -> None:
    cb = CircuitBreaker(config=CircuitConfig(max_consecutive_losses=3, cooldown_seconds=60))
    cb.record_loss()
    cb.record_loss()
    assert cb.state == "closed"
    cb.record_loss()
    assert cb.state == "open"
    assert cb.can_submit() is False


def test_win_resets_loss_counter() -> None:
    cb = CircuitBreaker(config=CircuitConfig(max_consecutive_losses=3))
    cb.record_loss()
    cb.record_loss()
    cb.record_win()
    cb.record_loss()
    cb.record_loss()
    assert cb.state == "closed"


def test_failed_orders_trip_open() -> None:
    cb = CircuitBreaker(config=CircuitConfig(max_failed_orders=2))
    cb.record_order_failure()
    cb.record_order_failure()
    assert cb.state == "open"


def test_cooldown_transitions_half_open() -> None:
    cb = CircuitBreaker(config=CircuitConfig(max_consecutive_losses=1, cooldown_seconds=0))
    cb.record_loss()
    assert cb.state == "open"
    # cooldown 0 -> permite half_open inmediato
    time.sleep(0.001)
    assert cb.can_submit() is True
    assert cb.state == "half_open"
    cb.confirm_recovery()
    assert cb.state == "closed"
