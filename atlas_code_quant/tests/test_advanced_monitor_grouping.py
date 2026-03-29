from __future__ import annotations

from atlas_code_quant.monitoring.advanced_monitor import NormalizedPosition, _infer_groups


def _equity(symbol: str, signed_qty: float) -> NormalizedPosition:
    return NormalizedPosition(
        symbol=symbol,
        underlying=symbol,
        asset_class="equity",
        signed_qty=signed_qty,
        quantity_abs=abs(signed_qty),
        side="short" if signed_qty < 0 else "long",
        strike=None,
        option_type=None,
        expiration=None,
        dte=None,
        entry_price=100.0,
        current_price=101.0,
        current_pnl=(101.0 - 100.0) * signed_qty,
        prev_close=99.0,
        greeks={},
    )


def test_infer_groups_classifies_single_long_equity() -> None:
    groups = _infer_groups([_equity("AAPL", 1.0)])

    assert len(groups) == 1
    assert groups[0]["strategy_type"] == "equity_long"


def test_infer_groups_classifies_single_short_equity() -> None:
    groups = _infer_groups([_equity("TSLA", -1.0)])

    assert len(groups) == 1
    assert groups[0]["strategy_type"] == "equity_short"
