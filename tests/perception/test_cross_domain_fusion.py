from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import (
    DealerPositioningSnapshot,
    FlowPerceptionSnapshot,
    InstitutionalOwnershipSnapshot,
    InsiderTradingSnapshot,
    MacroContextSnapshot,
    MarketPerceptionSnapshot,
    PoliticalTradingSnapshot,
    RegulatoryEventSnapshot,
)
from atlas_scanner.fusion import build_feature_vector


def test_cross_domain_fusion_builds_horizon_scores_and_domains() -> None:
    now = datetime.now(timezone.utc)
    market = MarketPerceptionSnapshot(
        symbol="SPY",
        timeframe="5m",
        as_of=now,
        spot_price=500.0,
        return_pct=0.4,
        momentum_score=62.0,
        relative_volume=1.3,
        volume_acceleration=0.15,
    )
    flow = FlowPerceptionSnapshot(
        symbol="SPY",
        timeframe="5m",
        as_of=now,
        call_premium=200_000.0,
        put_premium=90_000.0,
        dte_call_premium={"0-2": 120000.0},
        dte_put_premium={"0-2": 40000.0},
        net_gamma=0.8,
        oi_concentration=0.6,
    )
    dealer = DealerPositioningSnapshot(
        symbol="SPY",
        as_of=now,
        source="proxy",
        acceleration_zone=(498.0, 505.0),
        confidence=0.7,
        quality_flags={},
    )
    macro = MacroContextSnapshot(
        scope="SPY",
        as_of=now,
        source="stub",
        high_impact_flag=True,
        confidence=0.4,
        quality_flags={},
    )
    ownership = InstitutionalOwnershipSnapshot(
        symbol="SPY",
        as_of=now,
        source="stub",
        ownership_delta_pct=0.02,
        confidence=0.3,
        quality_flags={},
    )
    insider = InsiderTradingSnapshot(
        symbol="SPY",
        as_of=now,
        source="stub",
        buy_sell_ratio=1.1,
        confidence=0.3,
        quality_flags={},
    )
    political = PoliticalTradingSnapshot(
        scope="SPY",
        as_of=now,
        source="stub",
        net_political_flow=0.1,
        confidence=0.2,
        quality_flags={},
    )
    regulatory = RegulatoryEventSnapshot(
        symbol_or_scope="SPY",
        as_of=now,
        source="stub",
        severity="medium",
        overhang_score=0.2,
        confidence=0.2,
        quality_flags={},
    )
    vector = build_feature_vector(
        market,
        flow,
        dealer=dealer,
        macro=macro,
        ownership=ownership,
        insider=insider,
        political=political,
        regulatory=regulatory,
    )
    assert set(vector.horizon_scores.keys()) == {"intraday", "swing", "positional"}
    assert set(vector.freshness.keys()) == {"intraday", "swing", "positional"}
    assert "market" in vector.freshness["intraday"]
    assert set(vector.effective_weights.keys()) == {"intraday", "swing", "positional"}
    assert "macro" in vector.active_domains
    assert "ownership" in vector.active_domains
    assert "structural_confidence_score" in vector.meta
    assert "fast_pressure_score" in vector.meta
    assert "fast_structural_alignment" in vector.meta
    assert "horizon_conflict" in vector.meta
