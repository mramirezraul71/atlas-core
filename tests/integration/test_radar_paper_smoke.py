"""Smoke test interno: radar emite orden paper end-to-end (perfil paper_public_rest)."""

from __future__ import annotations

import pandas as pd
import pytest
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

from modules.atlas_radar_kalshi.brain import BrainDecision
from modules.atlas_radar_kalshi.config import reload_settings
from modules.atlas_radar_kalshi.orchestrator import Orchestrator
from modules.atlas_radar_kalshi.scanner import MarketEvent, OrderBookSnapshot


def _write_pem_2048(path) -> None:
    key = rsa.generate_private_key(
        public_exponent=65537, key_size=2048, backend=default_backend()
    )
    pem = key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption(),
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(pem)


@pytest.mark.asyncio
async def test_orchestrator_emits_paper_order_with_clear_edge(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """Libro con edge alto y brain fake: el ejecutor paper debe registrar intento."""
    secrets_dir = tmp_path / "secrets"
    logs_dir = tmp_path / "logs"
    pem_path = secrets_dir / "kalshi_private.pem"
    _write_pem_2048(pem_path)
    logs_dir.mkdir(parents=True, exist_ok=True)

    monkeypatch.setenv("ATLAS_RADAR_LIVE", "0")
    monkeypatch.setenv("ATLAS_RADAR_KILL", "0")
    monkeypatch.setenv("RADAR_PAPER_BALANCE_CENTS", "1000000")
    monkeypatch.setenv("RADAR_PROFILE", "paper_public_rest")
    monkeypatch.setenv("RADAR_LLM_BACKEND", "ollama")
    monkeypatch.setenv("KALSHI_API_KEY", "00000000-0000-0000-0000-000000000000")
    monkeypatch.setenv("KALSHI_PRIVATE_KEY_PATH", str(pem_path))
    monkeypatch.setenv("ATLAS_LOG_DIR", str(logs_dir))
    monkeypatch.setenv("RADAR_COOLDOWN_S", "0")

    reload_settings()
    orch = Orchestrator()
    orch.risk.update_balance(1_000_000)

    book = OrderBookSnapshot(
        market_ticker="TEST-MKT",
        yes_bids=[(38, 200), (37, 100)],
        yes_asks=[(40, 200), (41, 200)],
        no_bids=[(58, 200)],
        no_asks=[(62, 200)],
    )
    payload = dict(book.model_dump(mode="json"))
    ev = MarketEvent(kind="orderbook", market_ticker="TEST-MKT", payload=payload)

    async def fake_eval(ticker, book_snap, history):
        return BrainDecision(
            market_ticker=ticker,
            p_model=0.70,
            p_market=0.40,
            p_market_yes=0.40,
            p_market_no=0.60,
            edge=0.30,
            side="YES",
            confidence=0.85,
            mc_winrate=0.72,
            rationale="test_smoke_clear_edge",
        )

    orch.brain.evaluate = fake_eval  # type: ignore[method-assign]
    orch.scanner.history = lambda _t: pd.DataFrame({"yes_mid": [0.5] * 60})  # type: ignore[method-assign]

    assert hasattr(orch, "_on_event")
    await orch._on_event(ev)

    metrics = orch.executor.metrics.to_dict()
    assert metrics.get("attempts", 0) >= 1, (
        f"Radar no intentó enviar orden. metrics={metrics}"
    )
