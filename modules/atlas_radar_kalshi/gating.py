"""
gating.py — Filtros de calidad, prioridad y cooldown.

Decide si una oportunidad pasa a ser candidata operativa o se descarta.
Aplica:

- ``edge_net_min`` (incluye fees + slippage estimado).
- ``confidence_min``.
- ``spread_max_ticks``, ``min_depth_yes``, ``min_depth_no``.
- ``max_quote_age_ms``, ``max_latency_ms``.
- Cooldown por mercado (segundos desde última orden / decisión).
- Score operativo: ``edge_net * confidence * liquidity_score``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from pydantic import BaseModel, Field

from .signals import SignalReadout


class GateConfig(BaseModel):
    edge_net_min: float = 0.03
    confidence_min: float = 0.62
    spread_max_ticks: int = 5
    min_depth_yes: int = 50
    min_depth_no: int = 50
    max_quote_age_ms: int = 3000
    max_latency_ms: int = 1500
    cooldown_seconds: int = 30
    fee_per_contract_cents: float = 0.07   # ~7 bps de notional
    slippage_buffer_cents: float = 0.5     # 0.5¢


class GateDecision(BaseModel):
    accepted: bool
    reason: str = ""
    edge_gross: float = 0.0
    edge_net: float = 0.0
    score: float = 0.0
    side: Optional[str] = None
    price_cents: int = 50


@dataclass
class _Cooldowns:
    last_action: dict[str, float] = field(default_factory=dict)

    def ready(self, ticker: str, cooldown_s: int) -> bool:
        last = self.last_action.get(ticker, 0.0)
        return (time.time() - last) >= cooldown_s

    def stamp(self, ticker: str) -> None:
        self.last_action[ticker] = time.time()


class Gating:
    """Encapsula cooldowns + filtros + scoring."""

    def __init__(self, cfg: Optional[GateConfig] = None) -> None:
        self.cfg = cfg or GateConfig()
        self._cd = _Cooldowns()

    # ------------------------------------------------------------------
    def evaluate(
        self,
        ticker: str,
        readout: SignalReadout,
        p_market: float,
        quote_age_ms: int,
        latency_ms: int,
    ) -> GateDecision:
        cfg = self.cfg

        # 1) Datos stale / latencia
        if quote_age_ms > cfg.max_quote_age_ms:
            return GateDecision(accepted=False,
                                reason=f"quote_age={quote_age_ms}ms")
        if latency_ms > cfg.max_latency_ms:
            return GateDecision(accepted=False,
                                reason=f"latency={latency_ms}ms")

        # 2) Liquidez / spread
        if readout.spread_ticks > cfg.spread_max_ticks:
            return GateDecision(accepted=False,
                                reason=f"spread={readout.spread_ticks}")
        if readout.depth_yes < cfg.min_depth_yes:
            return GateDecision(accepted=False,
                                reason=f"depth_yes={readout.depth_yes}")
        if readout.depth_no < cfg.min_depth_no:
            return GateDecision(accepted=False,
                                reason=f"depth_no={readout.depth_no}")

        # 3) Cooldown
        if not self._cd.ready(ticker, cfg.cooldown_seconds):
            return GateDecision(accepted=False, reason="cooldown")

        # 4) Edge
        edge_gross = readout.p_ensemble - p_market
        side = "YES" if edge_gross >= 0 else "NO"
        # estimación de costo total (fees + slippage) en probabilidad
        cost_prob = (cfg.fee_per_contract_cents +
                     cfg.slippage_buffer_cents) / 100.0
        edge_net = abs(edge_gross) - cost_prob

        if readout.confidence < cfg.confidence_min:
            return GateDecision(accepted=False,
                                reason=f"confidence={readout.confidence:.3f}",
                                edge_gross=edge_gross, edge_net=edge_net)
        if edge_net < cfg.edge_net_min:
            return GateDecision(accepted=False,
                                reason=f"edge_net={edge_net:.4f}",
                                edge_gross=edge_gross, edge_net=edge_net)

        score = edge_net * readout.confidence * max(0.05, readout.liquidity_score)
        # precio operativo aproximado
        price_cents = int(round((p_market if side == "YES"
                                  else 1.0 - p_market) * 100))
        price_cents = max(1, min(99, price_cents))

        return GateDecision(
            accepted=True,
            reason="ok",
            edge_gross=edge_gross,
            edge_net=edge_net,
            score=score,
            side=side,
            price_cents=price_cents,
        )

    def stamp(self, ticker: str) -> None:
        self._cd.stamp(ticker)
