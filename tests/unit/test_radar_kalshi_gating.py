"""Tests del :mod:`modules.atlas_radar_kalshi.gating`.

Cubre:
- Aceptación con readout válido + edge suficiente.
- Rechazo por spread, depth, latencia, quote_age, confidence, edge.
- Cooldown por mercado.
- Cómputo edge_net (incluye fees + slippage).
"""
from __future__ import annotations

import time

import pytest

from modules.atlas_radar_kalshi.gating import GateConfig, Gating
from modules.atlas_radar_kalshi.signals import SignalReadout


def _readout(p: float = 0.7, conf: float = 0.8, spread: int = 2,
             dy: int = 200, dn: int = 200) -> SignalReadout:
    return SignalReadout(
        p_micro=p, p_markov=p, p_llm=p, p_momentum=p, p_ensemble=p,
        confidence=conf, liquidity_score=0.7, spread_ticks=spread,
        microprice=p * 100, depth_yes=dy, depth_no=dn,
    )


def _g(**over) -> Gating:
    base = dict(
        edge_net_min=0.03, confidence_min=0.6,
        spread_max_ticks=3, min_depth_yes=50, min_depth_no=50,
        max_quote_age_ms=2000, max_latency_ms=1000,
        cooldown_seconds=10,
    )
    base.update(over)
    return Gating(GateConfig(**base))


# ---------------------------------------------------------------------------
class TestAccept:
    def test_accept_basic(self) -> None:
        gate = _g().evaluate("M1", _readout(p=0.7),
                             p_market=0.5, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is True
        assert gate.side == "YES"
        assert gate.edge_gross > 0
        assert gate.edge_net > 0
        assert gate.score > 0

    def test_accept_no_side_when_p_below_market(self) -> None:
        gate = _g().evaluate("M1", _readout(p=0.3),
                             p_market=0.5, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is True
        assert gate.side == "NO"


# ---------------------------------------------------------------------------
class TestReject:
    def test_reject_quote_stale(self) -> None:
        gate = _g().evaluate("M1", _readout(),
                             p_market=0.4, quote_age_ms=5000, latency_ms=50)
        assert gate.accepted is False
        assert "quote_age" in gate.reason

    def test_reject_high_latency(self) -> None:
        gate = _g().evaluate("M1", _readout(),
                             p_market=0.4, quote_age_ms=100, latency_ms=2000)
        assert gate.accepted is False
        assert "latency" in gate.reason

    def test_reject_wide_spread(self) -> None:
        gate = _g().evaluate("M1", _readout(spread=10),
                             p_market=0.4, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is False
        assert "spread" in gate.reason

    def test_reject_thin_depth_yes(self) -> None:
        gate = _g().evaluate("M1", _readout(dy=10),
                             p_market=0.4, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is False
        assert "depth_yes" in gate.reason

    def test_reject_thin_depth_no(self) -> None:
        gate = _g().evaluate("M1", _readout(dn=10),
                             p_market=0.4, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is False
        assert "depth_no" in gate.reason

    def test_reject_low_confidence(self) -> None:
        gate = _g().evaluate("M1", _readout(conf=0.3),
                             p_market=0.4, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is False
        assert "confidence" in gate.reason

    def test_reject_low_edge(self) -> None:
        # p_market casi igual a p → edge insuficiente
        gate = _g().evaluate("M1", _readout(p=0.51),
                             p_market=0.50, quote_age_ms=100, latency_ms=50)
        assert gate.accepted is False
        assert "edge_net" in gate.reason


# ---------------------------------------------------------------------------
class TestCooldown:
    def test_cooldown_blocks_repeat(self) -> None:
        g = _g(cooldown_seconds=60)
        first = g.evaluate("M1", _readout(p=0.7), p_market=0.5,
                           quote_age_ms=100, latency_ms=50)
        assert first.accepted is True
        g.stamp("M1")
        second = g.evaluate("M1", _readout(p=0.7), p_market=0.5,
                            quote_age_ms=100, latency_ms=50)
        assert second.accepted is False
        assert second.reason == "cooldown"

    def test_cooldown_does_not_block_other_market(self) -> None:
        g = _g(cooldown_seconds=60)
        g.stamp("M1")
        out = g.evaluate("M2", _readout(p=0.7), p_market=0.5,
                         quote_age_ms=100, latency_ms=50)
        assert out.accepted is True


# ---------------------------------------------------------------------------
class TestEdgeNet:
    def test_edge_net_includes_costs(self) -> None:
        # edge_gross = 0.20, costo ≈ 0.6¢ → edge_net ≈ 0.194
        g = _g()
        out = g.evaluate("M1", _readout(p=0.70), p_market=0.50,
                         quote_age_ms=100, latency_ms=50)
        assert out.edge_gross == pytest.approx(0.20, abs=1e-9)
        assert out.edge_net < out.edge_gross
        assert out.edge_net > 0.18
