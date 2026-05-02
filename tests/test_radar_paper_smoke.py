"""Integración mínima paper: pipeline hasta submit sin scanner ni red."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import pandas as pd
import pytest

from modules.atlas_radar_kalshi.brain import BrainDecision
from modules.atlas_radar_kalshi.config import RadarSettings
from modules.atlas_radar_kalshi.orchestrator import Orchestrator
from modules.atlas_radar_kalshi.scanner import MarketEvent, OrderBookSnapshot


@pytest.mark.asyncio
async def test_paper_orchestrator_submit_increments_executor_attempts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("RADAR_PROFILE", "paper_aggressive")
    monkeypatch.setenv("RADAR_COOLDOWN_S", "0")
    monkeypatch.setenv("ATLAS_RADAR_LIVE", "0")
    # Aislar de .env del host (p. ej. RADAR_W_LLM=0) para que el ensemble no quede en ~0.5.
    monkeypatch.setenv("RADAR_EDGE_NET_MIN", "0.001")
    monkeypatch.setenv("RADAR_CONFIDENCE_MIN", "0.25")
    monkeypatch.setenv("RADAR_W_MICRO", "0.20")
    monkeypatch.setenv("RADAR_W_MARKOV", "0.15")
    monkeypatch.setenv("RADAR_W_LLM", "0.55")
    monkeypatch.setenv("RADAR_W_MOM", "0.10")

    log_dir = tmp_path / "logs"
    settings = RadarSettings(
        kalshi_api_key_id="smoke-key",
        kalshi_private_key_path=tmp_path / "nop.pem",
        log_dir=log_dir,
        kalshi_environment="demo",
    )
    orch = Orchestrator(settings=settings)
    orch.risk.update_balance(1_000_000)

    async def _fake_brain(ticker: str, book: OrderBookSnapshot, history: pd.DataFrame) -> BrainDecision:
        return BrainDecision(
            market_ticker=ticker,
            p_model=0.78,
            p_market=0.50,
            p_market_yes=0.52,
            p_market_no=0.48,
            edge=0.26,
            side="YES",
            confidence=0.92,
            mc_winrate=0.55,
            rationale="smoke",
        )

    orch.brain.evaluate = _fake_brain  # type: ignore[method-assign]
    orch.scanner.history = lambda _t: pd.DataFrame({"yes_mid": [0.5] * 60})  # type: ignore[method-assign]

    book = OrderBookSnapshot(
        market_ticker="SMOKE-T",
        yes_bids=[(48, 100)],
        yes_asks=[(52, 100)],
        no_bids=[(48, 100)],
        no_asks=[(52, 100)],
        ts=datetime.now(timezone.utc),
    )
    ev = MarketEvent(
        kind="orderbook",
        market_ticker="SMOKE-T",
        payload=book.model_dump(),
    )

    await orch._on_event(ev)

    assert orch.executor.metrics.attempts >= 1
