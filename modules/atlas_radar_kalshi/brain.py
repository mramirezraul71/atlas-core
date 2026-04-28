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
from typing import Dict, List, Optional

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
    p_market_yes: float = Field(0.5, ge=0.0, le=1.0)
    p_market_no: float = Field(0.5, ge=0.0, le=1.0)
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
    FALLBACK_MODELS = (
        "qwen2.5-coder:7b",
        "llama3.1:8b",
        "llama3.1:latest",
    )

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("brain", self.settings.log_dir,
                              self.settings.log_level)
        self._market_meta: Dict[str, dict] = {}
        self._ollama_backoff_until: float = 0.0
        self._active_ollama_model: str = self.settings.ollama_model

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

        # 5) Edge vs mercado por lado ejecutable (ask YES/NO).
        px = self._executable_probabilities(book)
        edge_yes = winrate - px["yes_ask"]
        edge_no = (1.0 - winrate) - px["no_ask"]
        if edge_yes >= edge_no:
            edge = edge_yes
            p_market = px["yes_ask"]
            side_candidate = "YES"
        else:
            edge = edge_no
            p_market = px["no_ask"]
            side_candidate = "NO"
        side: Optional[str]
        if edge >= self.settings.edge_threshold:
            side = side_candidate
        else:
            side = None

        decision = BrainDecision(
            market_ticker=ticker,
            p_model=winrate,
            p_market=p_market,
            p_market_yes=px["yes_ask"],
            p_market_no=px["no_ask"],
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

    @staticmethod
    def _best_price(side: list[tuple[int, int]], mode: str) -> Optional[int]:
        prices = [int(p) for p, q in side if int(q) > 0]
        if not prices:
            return None
        return max(prices) if mode == "max" else min(prices)

    def _executable_probabilities(self, book: OrderBookSnapshot) -> dict[str, float]:
        yes_ask_c = self._best_price(book.yes_asks, "min")
        yes_bid_c = self._best_price(book.yes_bids, "max")
        no_ask_c = self._best_price(book.no_asks, "min")
        no_bid_c = self._best_price(book.no_bids, "max")
        if yes_ask_c is None and no_bid_c is not None:
            yes_ask_c = max(1, min(99, 100 - no_bid_c))
        if no_ask_c is None and yes_bid_c is not None:
            no_ask_c = max(1, min(99, 100 - yes_bid_c))
        mid_yes = float(book.yes_mid if book.yes_mid is not None else 0.5)
        yes_ask = float((yes_ask_c if yes_ask_c is not None else int(round(mid_yes * 100))) / 100.0)
        no_ask = float((no_ask_c if no_ask_c is not None else int(round((1.0 - mid_yes) * 100))) / 100.0)
        return {
            "yes_ask": max(0.001, min(0.999, yes_ask)),
            "no_ask": max(0.001, min(0.999, no_ask)),
            "mid_yes": max(0.001, min(0.999, mid_yes)),
        }

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
        try:
            async with httpx.AsyncClient(
                timeout=self.settings.ollama_timeout_seconds
            ) as client:
                base_url = self.settings.ollama_endpoint.rstrip("/")
                installed = await self._fetch_installed_models(client, base_url)
                candidates = self._candidate_ollama_models(installed)
                data = {}
                last_error: Optional[str] = None
                for model in candidates:
                    try:
                        data = await self._call_ollama_with_fallback(
                            client=client,
                            base_url=base_url,
                            model=model,
                            prompt=prompt,
                        )
                        self._active_ollama_model = model
                        break
                    except httpx.HTTPStatusError as exc:
                        body = (exc.response.text or "").lower()
                        if exc.response.status_code == 404 and "model" in body:
                            last_error = (
                                f"model_not_found:{model}"
                            )
                            continue
                        raise
                    except Exception as exc:
                        last_error = str(exc)
                        continue
                if not data:
                    raise RuntimeError(
                        f"sin_respuesta_ollama; ultimo_error={last_error or 'n/a'}"
                    )
            self._ollama_backoff_until = 0.0
            return LLMReading(
                p_yes=float(data.get("p_yes", 0.5)),
                confidence=float(data.get("confidence", 0.3)),
                rationale=str(data.get("rationale", f"model={self._active_ollama_model}")),
            )
        except Exception as exc:
            self._ollama_backoff_until = (
                time.time() + float(self.settings.ollama_backoff_seconds)
            )
            self.log.warning("Ollama no disponible (%s); usando prior 0.5", exc)
            return LLMReading(p_yes=0.5, confidence=0.1, rationale=str(exc))

    def _candidate_ollama_models(self, installed: List[str]) -> List[str]:
        seen = set()
        ordered: List[str] = []
        base = [self._active_ollama_model, self.settings.ollama_model, *self.FALLBACK_MODELS]
        installed_set = set(installed)
        for model in base:
            name = (model or "").strip()
            if not name or name in seen:
                continue
            if installed and name not in installed_set:
                continue
            seen.add(name)
            ordered.append(name)
        if ordered:
            return ordered
        return installed[:3] if installed else [self.settings.ollama_model]

    async def _fetch_installed_models(
        self, client: httpx.AsyncClient, base_url: str
    ) -> List[str]:
        try:
            r = await client.get(f"{base_url}/api/tags")
            r.raise_for_status()
            models = r.json().get("models", [])
            names = [str(m.get("name", "")).strip() for m in models if m.get("name")]
            return [n for n in names if n]
        except Exception as exc:
            self.log.debug("No se pudo leer /api/tags de Ollama (%s)", exc)
            return []

    async def _call_ollama_with_fallback(
        self,
        client: httpx.AsyncClient,
        base_url: str,
        model: str,
        prompt: str,
    ) -> Dict[str, float | str]:
        generate_payload = {
            "model": model,
            "prompt": prompt,
            "format": "json",
            "stream": False,
            "keep_alive": "30m",
            "options": {"temperature": 0.2, "num_ctx": 2048, "num_predict": 120},
        }
        r = await client.post(f"{base_url}/api/generate", json=generate_payload)
        if r.status_code != 404:
            r.raise_for_status()
            return self._extract_json_payload(r.json().get("response", "{}"))

        # Compatibilidad con builds que exponen /api/chat pero no /api/generate.
        chat_payload = {
            "model": model,
            "messages": [{"role": "user", "content": prompt}],
            "format": "json",
            "stream": False,
            "keep_alive": "30m",
            "options": {"temperature": 0.2, "num_ctx": 2048, "num_predict": 120},
        }
        rc = await client.post(f"{base_url}/api/chat", json=chat_payload)
        rc.raise_for_status()
        msg = (rc.json().get("message") or {})
        raw = msg.get("content", "{}")
        return self._extract_json_payload(raw)

    @staticmethod
    def _extract_json_payload(raw: str) -> Dict[str, float | str]:
        text = (raw or "{}").strip()
        if text.startswith("```"):
            text = text.strip("`")
            if text.lower().startswith("json"):
                text = text[4:].strip()
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            text = text[start : end + 1]
        data = json.loads(text)
        return {
            "p_yes": float(data.get("p_yes", 0.5)),
            "confidence": float(data.get("confidence", 0.3)),
            "rationale": str(data.get("rationale", "")),
        }

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
