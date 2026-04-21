from __future__ import annotations

from atlas_code_quant.learning.seasonality_context import build_seasonality_context


def test_seasonality_context_structure() -> None:
    out = build_seasonality_context({"as_of_utc": "2026-04-21T14:35:00Z"})
    assert out["session"] in {"US", "premarket", "afterhours", "other"}
    assert out["hour_bucket"] in {"open", "mid", "close", "premarket", "afterhours", "off"}
    assert out["seasonal_bias"] in {"favorable", "neutral", "hostile", "unknown"}
    assert isinstance(out["is_month_end"], bool)
    assert isinstance(out["is_quarter_end"], bool)


def test_seasonality_context_flags_quarter_end_open() -> None:
    out = build_seasonality_context({"as_of_utc": "2026-03-30T13:10:00Z"})
    assert out["is_quarter_end"] is True
    assert out["seasonal_bias"] == "hostile"
