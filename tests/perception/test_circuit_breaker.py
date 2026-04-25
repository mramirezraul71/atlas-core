from __future__ import annotations

from atlas_scanner.perception.common.circuit_breaker import CircuitBreaker


def test_circuit_breaker_open_and_half_open_cycle() -> None:
    breaker = CircuitBreaker(name="test_provider", failure_threshold=2, cooldown_sec=1)
    assert breaker.allow_request() is True
    breaker.record_failure("err_1")
    assert breaker.snapshot().state == "closed"
    breaker.record_failure("err_2")
    assert breaker.snapshot().state == "open"
    assert breaker.allow_request() is False
    breaker._opened_at = breaker._opened_at - 2 if breaker._opened_at else None  # force cooldown pass
    assert breaker.allow_request() is True
    assert breaker.snapshot().state == "half_open"
    breaker.record_success()
    assert breaker.snapshot().state == "closed"
