from __future__ import annotations


def test_retry_policy_fixed_steps():
    from modules.humanoid.resilience.retry_policy import RetryPolicy

    p = RetryPolicy.fixed([5, 10, 30], jitter=0.0)
    assert p.delay_for_attempt(0) == 5
    assert p.delay_for_attempt(1) == 10
    assert p.delay_for_attempt(2) == 30
    assert p.delay_for_attempt(99) == 30


def test_retry_policy_exponential_cap():
    from modules.humanoid.resilience.retry_policy import RetryPolicy

    p = RetryPolicy(max_attempts=5, initial_delay=5, factor=3, max_delay=30, jitter=0.0)
    assert p.delay_for_attempt(0) == 5
    assert p.delay_for_attempt(1) == 15
    assert p.delay_for_attempt(2) == 30  # cap
    assert p.delay_for_attempt(3) == 30  # cap stays

