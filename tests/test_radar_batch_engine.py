"""Motor batch F2: scoring y ordenación."""
from __future__ import annotations

import asyncio
from unittest.mock import patch

import pytest

from atlas_adapter.services.radar_batch_engine import (
    build_radar_opportunities_batch,
    opportunity_classification,
    score_from_summary,
)
from atlas_adapter.services.universe_provider import UniverseEntry
from atlas_code_quant.config.feature_flags import AtlasFeatureFlags


def test_score_from_summary_conservative() -> None:
    body = {
        "radar": {
            "signal": {
                "meta": {
                    "snapshot_classification": "fully_operable",
                    "fast_pressure_score": 0.8,
                    "structural_confidence_score": 0.6,
                }
            }
        }
    }
    s = score_from_summary(body)
    assert 0.0 <= s <= 100.0
    assert s > 50.0


def test_opportunity_classification_threshold() -> None:
    assert opportunity_classification(40.0, 80.0, "fully_operable") == "below_threshold"
    assert opportunity_classification(90.0, 80.0, "fully_operable") == "high_conviction"


def test_batch_ranking_sync_wrapper() -> None:
    """Ejecuta coroutine sin depender de pytest-asyncio."""
    asyncio.run(_run_batch_ranking())


async def _run_batch_ranking() -> None:
    async def fake_sse(symbol: str):
        scores = {"ZZZ": 0.05, "AAA": 0.99, "B": 0.1, "A": 0.99}
        f = scores.get(symbol, 0.5)
        body = {
            "symbol": symbol,
            "last_update": "t",
            "radar": {
                "signal": {
                    "bias": "neutral",
                    "meta": {
                        "snapshot_classification": "fully_operable",
                        "fast_pressure_score": f,
                        "structural_confidence_score": 0.5,
                    },
                }
            },
            "degradations_active": [],
        }
        return body, False, None

    entries = [
        UniverseEntry("ZZZ", "equity_etf", "x"),
        UniverseEntry("AAA", "equity_etf", "x"),
    ]
    with patch(
        "atlas_adapter.services.radar_batch_engine.radar_build_dashboard_for_sse",
        new=fake_sse,
    ):
        batch = await build_radar_opportunities_batch(
            entries=entries,
            min_score=0.0,
            limit=5,
            asset_class=None,
            sector=None,
            flags=AtlasFeatureFlags(),
        )
    assert [o["symbol"] for o in batch.opportunities] == ["AAA", "ZZZ"]
