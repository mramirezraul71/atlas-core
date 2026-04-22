from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.scoring.offline import score_symbol


def _snapshot(
    symbol: str,
    *,
    liquidity_score: float,
    ref_price: float,
    event_risk: float,
    bid_ask_spread: float,
) -> SymbolSnapshot:
    return SymbolSnapshot(
        symbol=symbol,
        asset_type="equity",
        base_currency="USD",
        ref_price=ref_price,
        volatility_lookback=0.20,
        liquidity_score=liquidity_score,
        meta={"event_risk": event_risk, "bid_ask_spread": bid_ask_spread},
    )


def test_score_symbol_includes_explanation_strengths_penalties() -> None:
    scored = score_symbol(
        _snapshot(
            "MIXED",
            liquidity_score=0.90,
            ref_price=300.0,
            event_risk=0.95,
            bid_ask_spread=0.09,
        )
    )
    assert isinstance(scored.explanation, str) and scored.explanation != ""
    assert isinstance(scored.strengths, tuple)
    assert isinstance(scored.penalties, tuple)
    assert "high liquidity" in scored.strengths
    assert "elevated event risk" in scored.penalties


def test_explanation_uses_strengths_and_penalties_when_both_exist() -> None:
    scored = score_symbol(
        _snapshot(
            "BOTH",
            liquidity_score=0.85,
            ref_price=350.0,
            event_risk=0.90,
            bid_ask_spread=0.09,
        )
    )
    assert "Pros:" in scored.explanation
    assert "Risks:" in scored.explanation


def test_explanation_balanced_profile_when_no_extremes() -> None:
    scored = score_symbol(
        _snapshot(
            "BALANCED",
            liquidity_score=0.50,
            ref_price=250.0,
            event_risk=0.50,
            bid_ask_spread=0.05,
        )
    )
    assert scored.explanation == "Balanced profile with mixed component signals."


def test_strengths_and_penalties_follow_component_thresholds() -> None:
    scored = score_symbol(
        _snapshot(
            "BORDER",
            liquidity_score=0.70,   # strength boundary
            ref_price=153.5,        # price component ~= 0.30 penalty boundary
            event_risk=0.30,        # event risk component = 0.70 strength boundary
            bid_ask_spread=0.07,    # spread component = 0.30 penalty boundary
        )
    )
    assert "high liquidity" in scored.strengths
    assert "contained event risk" in scored.strengths
    assert "unfavorable price range" in scored.penalties
    assert "wide bid/ask spread" in scored.penalties

