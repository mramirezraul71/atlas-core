from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import DealerPositioningSnapshot, FlowPerceptionSnapshot, MarketPerceptionSnapshot
from atlas_scanner.fusion import build_feature_vector


def test_dealer_weight_is_higher_intraday_than_swing() -> None:
    now = datetime.now(timezone.utc)
    market = MarketPerceptionSnapshot(
        symbol="SPY",
        timeframe="5m",
        as_of=now,
        spot_price=500.0,
        return_pct=0.3,
        momentum_score=58.0,
        relative_volume=1.15,
        volume_acceleration=0.12,
    )
    flow = FlowPerceptionSnapshot(
        symbol="SPY",
        timeframe="5m",
        as_of=now,
        call_premium=150000.0,
        put_premium=90000.0,
        net_gamma=0.3,
        oi_concentration=0.5,
        dte_call_premium={"0-2": 100000.0},
        dte_put_premium={"0-2": 60000.0},
    )
    dealer = DealerPositioningSnapshot(
        symbol="SPY",
        as_of=now,
        source="options_chain",
        gamma_flip_level=502.0,
        call_wall=505.0,
        put_wall=498.0,
        pinning_zone=(499.0, 504.0),
        acceleration_zone=(500.0, 506.0),
        freshness_sec=45,
        delay_sec=0,
        confidence=0.8,
        quality_flags={"provider_ready": True},
        meta={"dealer_pressure_score": 61.0, "acceleration_direction": "bullish"},
    )
    vector = build_feature_vector(market, flow, dealer=dealer, as_of=now)
    assert vector.effective_weights["intraday"]["dealer"] >= vector.effective_weights["swing"]["dealer"]
    assert vector.freshness["positional"]["dealer"].status == "non_operable"
    dealer_meta = vector.meta.get("dealer_context")
    assert isinstance(dealer_meta, dict)
    assert dealer_meta.get("dealer_pressure_score") == 61.0
