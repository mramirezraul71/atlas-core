from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.fusion.freshness import evaluate_domain_freshness


def test_macro_freshness_ttl_reduced_in_volatility_window() -> None:
    now = datetime.now(timezone.utc)
    result = evaluate_domain_freshness(
        domain="macro",
        horizon="intraday",
        as_of=now,
        snapshot_as_of=now,
        delay_sec=0,
        confidence=0.8,
        quality_flags={"provider_ready": True, "calendar_volatility_window": True},
    )
    assert result.ttl_sec <= 180
    assert result.status == "usable"


def test_macro_freshness_ttl_extended_on_recent_surprise() -> None:
    now = datetime.now(timezone.utc)
    result = evaluate_domain_freshness(
        domain="macro",
        horizon="swing",
        as_of=now,
        snapshot_as_of=now,
        delay_sec=0,
        confidence=0.8,
        quality_flags={"provider_ready": True, "recent_surprise_significant": True},
    )
    assert result.ttl_sec >= 3600
