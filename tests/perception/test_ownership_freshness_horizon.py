from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import FlowPerceptionSnapshot, InstitutionalOwnershipSnapshot, MarketPerceptionSnapshot
from atlas_scanner.fusion import build_feature_vector


def test_ownership_not_operable_intraday_but_available_positional() -> None:
    now = datetime.now(timezone.utc)
    market = MarketPerceptionSnapshot(
        symbol="SPY",
        timeframe="1m",
        as_of=now,
        spot_price=500.0,
        return_pct=0.1,
        momentum_score=52.0,
        relative_volume=1.1,
        volume_acceleration=0.1,
    )
    flow = FlowPerceptionSnapshot(
        symbol="SPY",
        timeframe="1m",
        as_of=now,
        call_premium=100000.0,
        put_premium=80000.0,
        dte_call_premium={"0-2": 50000.0},
        dte_put_premium={"0-2": 40000.0},
    )
    ownership = InstitutionalOwnershipSnapshot(
        symbol="SPY",
        as_of=now,
        source="ownership_fmp",
        ownership_pct=24.0,
        ownership_delta_pct=0.3,
        concentration_score=0.55,
        freshness_sec=120 * 24 * 3600,
        delay_sec=45 * 24 * 3600,
        confidence=0.8,
        quality_flags={"provider_ready": True, "is_stub": False},
        meta={"sponsorship_score": 0.8, "ownership_signal": "bullish"},
    )
    vector = build_feature_vector(market, flow, ownership=ownership, as_of=now)
    assert vector.freshness["intraday"]["ownership"].status == "non_operable"
    assert vector.freshness["positional"]["ownership"].status in {"usable", "degraded"}
