from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from market_context import fundamental_score as fs # noqa: E402


def test_calculate_score_in_range(monkeypatch) -> None:
    monkeypatch.setattr(
        fs,
        "fetch_fundamental_data",
        lambda _sym: {
            "per": 12.0,
            "pbr": 2.0,
            "roe": 0.18,
            "debt_to_equity": 0.5,
            "dividend_yield": 0.03,
            "market_cap": 1e10,
        },
    )
    score = fs.calculate_fundamental_score("FAKE", sector_medians={"per": 20.0})
    assert 0 <= score <= 50


def test_filter_reject_below_15() -> None:
    assert fs.apply_fundamental_filter(10.0, min_score=25.0, reject_below=15.0) == "reject"


def test_filter_downweight_between(monkeypatch) -> None:
    assert fs.apply_fundamental_filter(20.0, min_score=25.0, reject_below=15.0) == "downweight"
    assert fs.apply_fundamental_filter(30.0, min_score=25.0, reject_below=15.0) == "accept"
