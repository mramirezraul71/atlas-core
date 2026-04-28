"""
signals.py — Ensemble de señales (microestructura + Markov + LLM).

Cada signal ∈ [0,1] estima ``p_yes`` desde un ángulo distinto. El
``SignalEnsemble`` combina linealmente con pesos configurables (env
``RADAR_W_*``) y devuelve también un ``confidence`` y un
``liquidity_score``.

Señales disponibles
-------------------
- ``micro``: precio medio implícito ponderado por profundidad
  (book-imbalance + microprice).
- ``markov``: distribución estacionaria de la cadena del histórico.
- ``llm``: probabilidad cruda devuelta por el modelo Ollama.
- ``momentum``: pendiente reciente de ``yes_mid`` (z-score corto).

Outputs auxiliares
------------------
- ``liquidity_score`` ∈ [0,1] = depth_total / max(depth_total).
- ``spread_ticks``: diferencia best_ask - best_bid (en ¢).
- ``microprice``: precio teórico ponderado por size opuesto.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd
from pydantic import BaseModel, Field

from .scanner import OrderBookSnapshot


class SignalReadout(BaseModel):
    p_micro: float = 0.5
    p_markov: float = 0.5
    p_llm: float = 0.5
    p_momentum: float = 0.5
    p_ensemble: float = 0.5
    confidence: float = 0.0
    liquidity_score: float = 0.0
    spread_ticks: int = 99
    microprice: float = 0.5
    depth_yes: int = 0
    depth_no: int = 0


@dataclass
class EnsembleWeights:
    micro: float = 0.30
    markov: float = 0.20
    llm: float = 0.40
    momentum: float = 0.10

    def normalized(self) -> "EnsembleWeights":
        s = self.micro + self.markov + self.llm + self.momentum
        if s <= 0:
            return EnsembleWeights(0.25, 0.25, 0.25, 0.25)
        return EnsembleWeights(self.micro / s, self.markov / s,
                               self.llm / s, self.momentum / s)


class SignalEnsemble:
    """Calcula la lectura ensemble a partir de book + history + LLM."""

    BUCKET_SIZE = 5

    def __init__(self, weights: Optional[EnsembleWeights] = None) -> None:
        self.weights = (weights or EnsembleWeights()).normalized()

    # ------------------------------------------------------------------
    @staticmethod
    def _best(book_side: list[tuple[int, int]],
              best: str = "max") -> tuple[Optional[int], int]:
        if not book_side:
            return None, 0
        prices = [(p, q) for p, q in book_side if q > 0]
        if not prices:
            return None, 0
        if best == "max":
            p, q = max(prices, key=lambda x: x[0])
        else:
            p, q = min(prices, key=lambda x: x[0])
        return p, q

    # ------------------------------------------------------------------
    def _micro(self, book: OrderBookSnapshot) -> tuple[float, float, int, int, int]:
        """Microprice + spread + depths."""
        bb_p, bb_q = self._best(book.yes_bids, "max")
        ba_p, ba_q = self._best(book.yes_asks, "min")
        depth_yes = sum(q for _, q in book.yes_bids)
        depth_no = sum(q for _, q in book.no_bids)
        # REST público / libros a una cara: sin esto spread=99 y el gating
        # rechaza siempre (spread_max ~5).
        if bb_p is None and ba_p is None:
            return 0.5, 0.5, 99, depth_yes, depth_no
        if bb_p is None:
            _ba = int(ba_p) if ba_p is not None else 50
            bb_p = max(1, _ba - 1)
            bb_q = max(1, int(bb_q or ba_q or 1))
        if ba_p is None:
            _bb = int(bb_p) if bb_p is not None else 50
            ba_p = min(99, _bb + 1)
            ba_q = max(1, int(ba_q or bb_q or 1))
        spread = max(0, int(ba_p) - int(bb_p))
        # microprice clásico: (ask*bid_size + bid*ask_size) / (bid_size+ask_size)
        denom = max(1, bb_q + ba_q)
        micro = (float(ba_p) * bb_q + float(bb_p) * ba_q) / denom
        # imbalance shrinkage hacia [0.05, 0.95]
        p = float(np.clip(micro / 100.0, 0.01, 0.99))
        return p, p, spread, depth_yes, depth_no

    def _markov(self, hist: pd.DataFrame) -> float:
        if hist is None or len(hist) < 30:
            return 0.5
        prices = (hist["yes_mid"].dropna() * 100).round().astype(int)
        if prices.empty:
            return 0.5
        buckets = (prices // self.BUCKET_SIZE).clip(0, 19)
        n = 20
        T = np.zeros((n, n))
        for a, b in zip(buckets[:-1], buckets[1:]):
            T[a, b] += 1
        rows = T.sum(axis=1, keepdims=True)
        rows[rows == 0] = 1
        T = T / rows
        v = np.full(n, 1 / n)
        for _ in range(150):
            v = v @ T
        states = (np.arange(n) + 0.5) * self.BUCKET_SIZE / 100.0
        return float(np.clip((v * states).sum(), 0.01, 0.99))

    @staticmethod
    def _momentum(hist: pd.DataFrame) -> float:
        if hist is None or len(hist) < 10:
            return 0.5
        x = hist["yes_mid"].dropna().to_numpy()[-30:]
        if len(x) < 5:
            return 0.5
        slope = (x[-1] - x[0]) / max(1, len(x))
        # squash a probabilidad
        return float(np.clip(0.5 + slope * 5, 0.01, 0.99))

    # ------------------------------------------------------------------
    def evaluate(
        self,
        book: OrderBookSnapshot,
        history: pd.DataFrame,
        p_llm: float,
        llm_confidence: float,
    ) -> SignalReadout:
        p_micro, micro_price, spread, depth_yes, depth_no = self._micro(book)
        p_markov = self._markov(history)
        p_mom = self._momentum(history)

        w = self.weights
        p_ens = (w.micro * p_micro + w.markov * p_markov +
                 w.llm * float(np.clip(p_llm, 0.01, 0.99)) +
                 w.momentum * p_mom)
        p_ens = float(np.clip(p_ens, 0.01, 0.99))

        # confidence:
        # - alta cuando las señales individuales coinciden,
        # - escala con la confianza del LLM y el book depth.
        signals = np.array([p_micro, p_markov, p_llm, p_mom])
        agreement = 1.0 - float(np.std(signals)) * 2.0
        agreement = max(0.0, min(1.0, agreement))
        liquidity = min(1.0, (depth_yes + depth_no) / 2000.0)
        confidence = 0.4 * agreement + 0.4 * float(llm_confidence) + 0.2 * liquidity

        return SignalReadout(
            p_micro=p_micro, p_markov=p_markov, p_llm=p_llm,
            p_momentum=p_mom, p_ensemble=p_ens,
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            liquidity_score=liquidity, spread_ticks=int(spread),
            microprice=float(micro_price),
            depth_yes=depth_yes, depth_no=depth_no,
        )
