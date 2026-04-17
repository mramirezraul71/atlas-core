from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.opportunity_scanner import _apply_runtime_ml_ranker


class _FakeRanker:
    def __init__(self, mapping: dict[str, float | None]) -> None:
        self.mapping = dict(mapping)
        self.calls: list[str] = []

    def score_runtime(self, ctx):
        self.calls.append(ctx.symbol)
        value = self.mapping.get(ctx.symbol)
        if value is None:
            return None, "ml_ranker_unavailable"
        return float(value), "scored"


def _candidate(symbol: str, selection_score: float, *, direction: str = "alcista") -> dict:
    return {
        "symbol": symbol,
        "asset_class": "equity_stock",
        "timeframe": "1h",
        "direction": direction,
        "price": 100.0,
        "strategy_key": "breakout_donchian",
        "strategy_type": "equity_long",
        "market_regime": "trending_strong",
        "selection_score": selection_score,
        "predicted_move_pct": 1.5,
        "order_flow": {"net_pressure_pct": 4.0},
    }


def test_runtime_uses_ml_ranker_when_trained() -> None:
    ranker = _FakeRanker({"AAA": 0.91, "BBB": 0.43})
    candidates = [
        _candidate("AAA", 80.0),
        _candidate("BBB", 80.0),
    ]

    enriched = _apply_runtime_ml_ranker(candidates, ranker=ranker, enabled=True)

    assert ranker.calls == ["AAA", "BBB"]
    assert enriched[0]["symbol"] == "AAA"
    assert enriched[0]["ml_score"] == 0.91
    assert enriched[1]["ml_score"] == 0.43


def test_runtime_degrades_gracefully_when_model_missing() -> None:
    candidates = [
        _candidate("AAA", 82.0),
        _candidate("BBB", 79.0),
    ]

    enriched = _apply_runtime_ml_ranker(candidates, ranker=None, enabled=True)

    assert [row["symbol"] for row in enriched] == ["AAA", "BBB"]
    assert all(row.get("ml_score") is None for row in enriched)
    assert all(row.get("ml_ranker_reason") == "ml_ranker_unavailable" for row in enriched)


def test_runtime_preserves_selection_without_ml_backend() -> None:
    ranker = _FakeRanker({"AAA": 0.10, "BBB": 0.99})
    candidates = [
        _candidate("AAA", 82.0),
        _candidate("BBB", 79.0),
    ]

    enriched = _apply_runtime_ml_ranker(candidates, ranker=ranker, enabled=False)

    assert ranker.calls == []
    assert [row["symbol"] for row in enriched] == ["AAA", "BBB"]
    assert all("ml_score" not in row for row in enriched)


def test_ml_ranker_score_is_exposed_in_candidate_payload() -> None:
    ranker = _FakeRanker({"AAA": 0.77})
    candidates = [_candidate("AAA", 84.0)]

    enriched = _apply_runtime_ml_ranker(candidates, ranker=ranker, enabled=True)

    assert enriched[0]["symbol"] == "AAA"
    assert enriched[0]["ml_score"] == 0.77
    assert enriched[0]["ml_ranker_reason"] == "scored"
