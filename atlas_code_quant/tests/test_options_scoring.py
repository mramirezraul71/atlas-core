from __future__ import annotations

import pytest

from atlas_code_quant.options_scoring import (
    GexData,
    GlobalRegime,
    Leg,
    OiData,
    OptionStructure,
    PriceRegime,
    VolData,
    calculate_gamma_score,
    calculate_oi_score,
    calculate_price_score,
    calculate_vol_score,
    combine_scores,
    get_min_total_score_for_asset_family,
    matches_context_strategy_map,
)


def test_scoring_components_and_weighted_score():
    regime = GlobalRegime(event_risk=False, regime_id="normal")
    vol = VolData(iv_rank=62.0, vrp_20d=9.5, term_structure_slope=1.1, skew_25d=6.0)
    gex = GexData(net_gex=-120.0, gamma_flip_distance_pct=1.0, max_pain_distance_pct=1.2)
    oi = OiData(oi_change_1d_pct=12.0, call_put_volume_ratio=1.0, max_pain_distance_pct=0.8)
    price = PriceRegime(adx=22.0, ema_alignment="mixed", breakout_status="range")

    v = calculate_vol_score(vol, regime)
    g = calculate_gamma_score(gex, regime)
    o = calculate_oi_score(oi, regime)
    p = calculate_price_score(price, regime)
    total = combine_scores(vol_score=v, gamma_score=g, oi_score=o, price_score=p, asset_family="INDEX")

    assert v > 70
    assert g > 70
    assert o > 70
    assert p > 60
    assert total >= 75


def test_matches_context_strategy_map_blocks_short_premium_event_driven_earnings():
    regime = GlobalRegime(event_risk=False, regime_id="normal")
    vol = VolData(iv_rank=70.0, vrp_20d=9.0, term_structure_slope=0.8, skew_25d=5.0)
    gex = GexData(net_gex=-20.0, gamma_flip_distance_pct=0.9, max_pain_distance_pct=0.7)
    oi = OiData(oi_change_1d_pct=15.0, call_put_volume_ratio=1.0, max_pain_distance_pct=1.0)
    price = PriceRegime(adx=20.0, ema_alignment="mixed", breakout_status="range")
    ok = matches_context_strategy_map(
        strategy="IRON_CONDOR",
        vol_data=vol,
        gex_data=gex,
        oi_data=oi,
        price_regime=price,
        global_regime=regime,
        asset_family="EVENT_DRIVEN_EQUITY",
        earnings_days=2,
        spread_pct=0.02,
    )
    assert ok is False


def test_option_structure_pydantic_validation():
    legs = [
        Leg(side="buy", type="call", strike=100, delta=0.55),
        Leg(side="sell", type="call", strike=105, delta=0.30),
    ]
    structure = OptionStructure(symbol="SPY", strategy="CALL_DEBIT_SPREAD", expiry="2026-06-19", legs=legs, contracts=2)
    assert structure.contracts == 2
    with pytest.raises(ValueError):
        OptionStructure(symbol="SPY", strategy="BROKEN", expiry="2026-06-19", legs=[legs[0]], contracts=1)


def test_min_total_score_config_lookup():
    assert get_min_total_score_for_asset_family("INDEX") == 75
    assert get_min_total_score_for_asset_family("HIGH_BETA_EQUITY") == 84
    assert get_min_total_score_for_asset_family("UNKNOWN_FAMILY") == 80