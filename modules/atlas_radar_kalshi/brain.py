"""
brain.py — Capa 2: Cerebro local del Radar.

Combina tres motores para estimar la probabilidad real de éxito de
un contrato Kalshi y producir una :class:`BrainDecision`:

1. **Sentimiento / razonamiento (Ollama)**
   Llama al endpoint local ``/api/generate`` (o ``/api/chat``) con un
   prompt estructurado que incluye ticker, descripción del evento y
   noticias recientes. El modelo devuelve un JSON con
   ``{p_yes, confidence, rationale}``.

2. **Cadena de Markov (1er orden) sobre buckets de precio**
   Construye una matriz de transición a partir del histórico de
   ``yes_mid`` discretizado en buckets de 5¢ y proyecta la
   distribución estacionaria.

3. **Simulación Monte Carlo**
   Combina la opinión del LLM con la cadena de Markov mediante un
   *soft-mixture* y simula ``N`` trayectorias hasta el cierre del
   mercado para validar p(YES) antes de disparar la orden.

La salida final es :class:`BrainDecision` con:

- ``p_model``: probabilidad estimada de YES.
- ``p_market``: probabilidad implícita actual.
- ``edge``: ``p_model - p_market``.
- ``side``: 'YES' / 'NO' / None.
- ``confidence``: 0-1.
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import httpx
import numpy as np
import pandas as pd
from pydantic import BaseModel, Field

from .config import RadarSettings, get_settings
from .scanner import MarketEvent, OrderBookSnapshot
from .utils.logger import get_logger


# ===========================================================================
# DTOs
# ===========================================================================
class LLMReading(BaseModel):
    p_yes: float = Field(..., ge=0.0, le=1.0)
    confidence: float = Field(..., ge=0.0, le=1.0)
    rationale: str = ""


class BrainDecision(BaseModel):
    market_ticker: str
    p_model: float = Field(..., ge=0.0, le=1.0)
    p_market: float = Field(..., ge=0.0, le=1.0)
    edge: float
    side: Optional[str] = Field(
        default=None,
        description="'YES', 'NO' o None si no hay edge suficiente.",
    )
    confidence: float = Field(0.0, ge=0.0, le=1.0)
    mc_winrate: float = Field(0.0, ge=0.0, le=1.0)
    rationale: str = ""

    def actionable(self, threshold: float) -> bool:
        return abs(self.edge) >= threshold and self.side is not None


# ===========================================================================
# Brain
# ===========================================================================
class RadarBrain:
    """Coordina LLM + Markov + Monte Carlo."""

    BUCKET_SIZE = 5  # cents

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("brain", self.settings.log_dir,
                              self.settings.log_level)
        self._market_meta: Dict[str, dict] = {}
        self._ollama_backoff_until: float = 0.0

    # ------------------------------------------------------------------
    # Entrada de eventos del scanner
    # ------------------------------------------------------------------
    def update_market_meta(self, ticker: str, payload: dict) -> None:
        """Actualiza metadata (título, cierre, categoría)."""
        self._market_meta[ticker] = payload

    async def evaluate(
        self,
        ticker: str,
        book: OrderBookSnapshot,
        history: pd.DataFrame,
        news_context: str = "",
    ) -> BrainDecision:
        """
        Pipeline completo:
            LLM(p1) ⊕ Markov(p2) → p_mix → Monte Carlo → BrainDecision
        """
        meta = self._market_meta.get(ticker, {})

        # 1) LLM
        llm = await self._ask_ollama(ticker, meta, news_context, book, history)

        # 2) Markov (offload CPU para no bloquear el event loop principal)
        p_markov = await asyncio.to_thread(self._markov_p_yes, history)

        # 3) Mix ponderado por confianza del LLM
        w_llm = 0.4 + 0.6 * llm.confidence  # 0.4..1.0
        p_mix = w_llm * llm.p_yes + (1 - w_llm) * p_markov
        p_mix = float(np.clip(p_mix, 0.001, 0.999))

        # 4) Monte Carlo (offload CPU para mantener responsivo /health,/ui)
        winrate = await asyncio.to_thread(self._monte_carlo, p_mix, history)

        # 5) Edge vs mercado
        p_market = book.yes_mid if book.yes_mid is not None else 0.5
        edge = winrate - p_market
        side: Optional[str]
        if edge >= self.settings.edge_threshold:
            side = "YES"
        elif -edge >= self.settings.edge_threshold:
            side = "NO"
        else:
            side = None

        decision = BrainDecision(
            market_ticker=ticker,
            p_model=winrate,
            p_market=p_market,
            edge=edge,
            side=side,
            confidence=llm.confidence,
            mc_winrate=winrate,
            rationale=llm.rationale[:500],
        )
        log_fn = self.log.info if decision.side else self.log.debug
        log_fn(
            "Decision %s | p_model=%.3f p_mkt=%.3f edge=%+.3f side=%s",
            ticker,
            decision.p_model,
            decision.p_market,
            decision.edge,
            decision.side,
        )
        return decision

    # ------------------------------------------------------------------
    # 1) Ollama
    # ------------------------------------------------------------------
    async def _ask_ollama(
        self, ticker: str, meta: dict, news: str,
        book: OrderBookSnapshot, history: pd.DataFrame,
    ) -> LLMReading:
        now = time.time()
        if now < self._ollama_backoff_until:
            return LLMReading(
                p_yes=0.5,
                confidence=0.1,
                rationale="ollama_backoff_active",
            )

        prompt = self._build_prompt(ticker, meta, news, book, history)
        payload = {
            "model": self.settings.ollama_model,
            "prompt": prompt,
            "format": "json",
            "stream": False,
            "options": {"temperature": 0.2, "num_ctx": 8192},
        }
        url = f"{self.settings.ollama_endpoint}/api/generate"
        try:
            async with httpx.AsyncClient(
                timeout=self.settings.ollama_timeout_seconds
            ) as client:
                r = await client.post(url, json=payload)
                r.raise_for_status()
                raw = r.json().get("response", "{}")
            data = json.loads(raw)
            self._ollama_backoff_until = 0.0
            return LLMReading(
                p_yes=float(data.get("p_yes", 0.5)),
                confidence=float(data.get("confidence", 0.3)),
                rationale=str(data.get("rationale", "")),
            )
        except Exception as exc:
            self._ollama_backoff_until = (
                time.time() + float(self.settings.ollama_backoff_seconds)
            )
            self.log.warning("Ollama no disponible (%s); usando prior 0.5", exc)
            return LLMReading(p_yes=0.5, confidence=0.1, rationale=str(exc))

    @staticmethod
    def _build_prompt(ticker: str, meta: dict, news: str,
                      book: OrderBookSnapshot, hist: pd.DataFrame) -> str:
        title = meta.get("title") or meta.get("subtitle") or ticker
        close = meta.get("close_time") or meta.get("expiration_time") or "n/a"
        last = float(hist["yes_mid"].iloc[-1]) if len(hist) else book.yes_mid
        return (
            "Eres un analista cuantitativo experto en mercados de eventos "
            "(Kalshi). Devuelve EXCLUSIVAMENTE un JSON válido con las "
            "claves {p_yes, confidence, rationale}.\n"
            f"Mercado: {ticker}\nTítulo: {title}\nCierre: {close}\n"
            f"Precio YES actual (prob.): {last}\n"
            f"Contexto / noticias: {news[:1500]}\n"
            "Estima la probabilidad real de que el evento se resuelva YES, "
            "tu confianza (0-1) y un rationale corto. JSON:"
        )

    # ------------------------------------------------------------------
    # 2) Cadena de Markov de 1er orden
    # ------------------------------------------------------------------
    def _markov_p_yes(self, hist: pd.DataFrame) -> float:
        if hist is None or len(hist) < 30:
            return 0.5
        prices = (hist["yes_mid"].dropna() * 100).round().astype(int)
        if prices.empty:
            return 0.5
        buckets = (prices // self.BUCKET_SIZE).clip(0, 19)  # 20 estados (0..95¢)
        n = 20
        T = np.zeros((n, n))
        for a, b in zip(buckets[:-1], buckets[1:]):
            T[a, b] += 1
        rows = T.sum(axis=1, keepdims=True)
        rows[rows == 0] = 1
        T = T / rows

        # Distribución estacionaria por iteración de potencia
        v = np.full(n, 1 / n)
        for _ in range(200):
            v = v @ T
        # E[YES] = sum(p_state * mid_price_state) / 100
        states = (np.arange(n) + 0.5) * self.BUCKET_SIZE / 100.0
        return float(np.clip((v * states).sum(), 0.01, 0.99))

    # ------------------------------------------------------------------
    # 3) Monte Carlo
    # ------------------------------------------------------------------
    def _monte_carlo(self, p_mix: float, hist: pd.DataFrame) -> float:
        """
        Simula N caminatas hasta el cierre. Modelo simple:
        - GBM sobre ``logit(p_mix)`` con sigma estimada del histórico.
        - El "ganador" es YES si el precio final cruza > 0.5.
        """
        N = self.settings.monte_carlo_iters
        steps = 100  # discretización hasta vencimiento
        # sigma a partir del histórico (en log-odds)
        if hist is not None and len(hist) > 5:
            x = (hist["yes_mid"].clip(0.01, 0.99)).to_numpy()
            logit = np.log(x / (1 - x))
            sigma = float(np.std(np.diff(logit)) or 0.05)
        else:
            sigma = 0.05

        mu_logit = math.log(p_mix / (1 - p_mix))
        rng = np.random.default_rng()
        # caminata aleatoria con drift cero centrada en mu_logit
        shocks = rng.normal(0.0, sigma, size=(N, steps))
        paths = mu_logit + np.cumsum(shocks, axis=1)
        finals = paths[:, -1]
        p_final = 1 / (1 + np.exp(-finals))
        wins = (p_final > 0.5).mean()
        return float(np.clip(wins, 0.01, 0.99))
