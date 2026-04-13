"""Sentiment Feed — Fase 0 Semana 2.

Calcula un score de sentimiento de mercado usando VADER sobre titulares
de fuentes públicas (Reddit/RSS financiero). El score se escribe en el
state de OperationCenter para que strategy_selector lo use como filtro.

Diseño deliberadamente simple:
- Sin API keys externas requeridas (usa RSS público de Reddit)
- Sin dependencias pesadas: solo vaderSentiment (puro Python)
- Feature flag: sentiment_feed_enabled en operation_center_state.json
- Actualización: llamar refresh() cada hora o al inicio de sesión

Integración:
    from atlas_code_quant.data.sentiment_feed import SentimentFeed
    feed = SentimentFeed(db_path=settings.journal_db_path)
    result = feed.refresh()
    # Escribe sentiment_score y sentiment_source en state automáticamente
"""
from __future__ import annotations

import json
import logging
import re
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger("quant.data.sentiment_feed")

# ── Constantes ────────────────────────────────────────────────────────────────

# Feeds RSS públicos — sin API key
_RSS_FEEDS: list[dict] = [
    {
        "name": "reddit_wallstreetbets",
        "url": "https://www.reddit.com/r/wallstreetbets/hot.json?limit=25",
        "type": "reddit_json",
    },
    {
        "name": "reddit_stocks",
        "url": "https://www.reddit.com/r/stocks/hot.json?limit=25",
        "type": "reddit_json",
    },
    {
        "name": "reddit_options",
        "url": "https://www.reddit.com/r/options/hot.json?limit=15",
        "type": "reddit_json",
    },
]

_REQUEST_TIMEOUT = 8          # segundos
_USER_AGENT = "ATLASBot/1.0 (paper trading research; +atlas_code_quant)"
_SCORE_SMOOTHING = 0.3        # EMA alpha para suavizar cambios bruscos
_NEUTRAL_SCORE = 0.0
_MIN_TEXTS = 3                # mínimo de textos para considerar el score válido


# ── Resultado ─────────────────────────────────────────────────────────────────

@dataclass
class SentimentResult:
    score: float                # [-1.0, 1.0] — negativo bearish, positivo bullish
    source: str                 # descripción de la fuente
    texts_analyzed: int
    positive_pct: float
    negative_pct: float
    neutral_pct: float
    timestamp: str
    error: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        return self.error is None and self.texts_analyzed >= _MIN_TEXTS


# ── Analizador VADER ligero ────────────────────────────────────────────────────

def _vader_available() -> bool:
    try:
        from vaderSentiment.vaderSentiment import SentimentIntensityAnalyzer  # noqa: F401
        return True
    except ImportError:
        return False


def _analyze_texts(texts: list[str]) -> dict:
    """Analiza una lista de textos con VADER y devuelve estadísticas agregadas."""
    if not texts:
        return {"compound": 0.0, "pos": 0.0, "neg": 0.0, "neu": 1.0, "count": 0}

    try:
        from vaderSentiment.vaderSentiment import SentimentIntensityAnalyzer
        analyzer = SentimentIntensityAnalyzer()
    except ImportError:
        # Fallback muy básico: contar palabras positivas/negativas
        logger.warning("vaderSentiment no disponible — usando fallback keyword")
        return _keyword_fallback(texts)

    compounds, pos_list, neg_list, neu_list = [], [], [], []
    for text in texts:
        scores = analyzer.polarity_scores(text)
        compounds.append(scores["compound"])
        pos_list.append(scores["pos"])
        neg_list.append(scores["neg"])
        neu_list.append(scores["neu"])

    n = len(compounds)
    return {
        "compound": sum(compounds) / n,
        "pos": sum(pos_list) / n,
        "neg": sum(neg_list) / n,
        "neu": sum(neu_list) / n,
        "count": n,
    }


def _keyword_fallback(texts: list[str]) -> dict:
    """Fallback si VADER no está instalado — keywords simples."""
    bullish_kw = {"bull", "buy", "long", "moon", "breakout", "surge", "rally", "gain", "up"}
    bearish_kw = {"bear", "sell", "short", "crash", "dump", "drop", "fall", "loss", "down"}

    pos_count = neg_count = 0
    for text in texts:
        words = set(re.findall(r"\w+", text.lower()))
        pos_count += len(words & bullish_kw)
        neg_count += len(words & bearish_kw)

    total = pos_count + neg_count or 1
    compound = (pos_count - neg_count) / max(total, 1)
    compound = max(-1.0, min(1.0, compound))
    pos_r = pos_count / total
    neg_r = neg_count / total

    return {
        "compound": compound,
        "pos": pos_r,
        "neg": neg_r,
        "neu": 1.0 - pos_r - neg_r,
        "count": len(texts),
    }


# ── Fetcher Reddit JSON ────────────────────────────────────────────────────────

def _fetch_reddit_texts(url: str) -> list[str]:
    """Extrae títulos de posts del endpoint JSON de Reddit."""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": _USER_AGENT})
        with urllib.request.urlopen(req, timeout=_REQUEST_TIMEOUT) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        posts = data.get("data", {}).get("children", [])
        titles = [
            p["data"].get("title", "") + " " + p["data"].get("selftext", "")[:200]
            for p in posts
            if p.get("data", {}).get("title")
        ]
        return [t.strip() for t in titles if t.strip()]
    except Exception as exc:
        logger.warning("Error fetch %s: %s", url, exc)
        return []


# ── Clase principal ────────────────────────────────────────────────────────────

class SentimentFeed:
    """Calcula y persiste el score de sentimiento de mercado.

    Args:
        state_path: Ruta al operation_center_state.json para escribir el score.
        cache_ttl_seconds: Tiempo mínimo entre actualizaciones reales (evita spam).
    """

    def __init__(
        self,
        state_path: Optional[Path] = None,
        cache_ttl_seconds: int = 3600,
    ) -> None:
        if state_path is None:
            # Ruta por defecto relativa a este archivo
            base = Path(__file__).parent.parent
            state_path = base / "data" / "operation" / "operation_center_state.json"
        self.state_path = Path(state_path)
        self.cache_ttl = cache_ttl_seconds
        self._last_score: float = _NEUTRAL_SCORE
        self._last_refresh: float = 0.0

    # ── Público ───────────────────────────────────────────────────────────────

    def refresh(self, force: bool = False) -> SentimentResult:
        """Actualiza el score de sentimiento.

        Respeta el TTL para no saturar fuentes. Usa force=True para ignorarlo.
        Escribe sentiment_score y sentiment_source en el state JSON.
        """
        now = time.time()
        if not force and (now - self._last_refresh) < self.cache_ttl:
            logger.debug("Sentiment cache válido — omitiendo refresh")
            return self._build_cached_result()

        all_texts: list[str] = []
        sources_ok: list[str] = []

        for feed in _RSS_FEEDS:
            if feed["type"] == "reddit_json":
                texts = _fetch_reddit_texts(feed["url"])
                if texts:
                    all_texts.extend(texts)
                    sources_ok.append(feed["name"])

        if not all_texts:
            logger.warning("No se pudieron obtener textos para sentiment — usando neutro")
            result = SentimentResult(
                score=_NEUTRAL_SCORE,
                source="unavailable",
                texts_analyzed=0,
                positive_pct=0.0,
                negative_pct=0.0,
                neutral_pct=1.0,
                timestamp=datetime.now(timezone.utc).isoformat(),
                error="No data sources available",
            )
            return result

        stats = _analyze_texts(all_texts)
        raw_score = float(stats["compound"])

        # EMA suavizado para evitar saltos bruscos entre refreshes
        smoothed = _SCORE_SMOOTHING * raw_score + (1 - _SCORE_SMOOTHING) * self._last_score
        self._last_score = smoothed
        self._last_refresh = now

        source_label = "vader:" + "+".join(sources_ok) if _vader_available() else "keyword:" + "+".join(sources_ok)

        result = SentimentResult(
            score=round(smoothed, 4),
            source=source_label,
            texts_analyzed=stats["count"],
            positive_pct=round(stats["pos"], 3),
            negative_pct=round(stats["neg"], 3),
            neutral_pct=round(stats["neu"], 3),
            timestamp=datetime.now(timezone.utc).isoformat(),
        )

        self._write_to_state(result)
        logger.info(
            "Sentiment actualizado: score=%.4f source=%s texts=%d",
            result.score, result.source, result.texts_analyzed,
        )
        return result

    def get_score(self) -> float:
        """Devuelve el último score calculado (sin hacer refresh)."""
        return self._last_score

    # ── Privado ───────────────────────────────────────────────────────────────

    def _write_to_state(self, result: SentimentResult) -> None:
        """Escribe sentiment_score y sentiment_source en el state JSON."""
        if not self.state_path.exists():
            logger.warning("State path no existe: %s — no se escribe sentiment", self.state_path)
            return
        try:
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            state["sentiment_score"] = result.score
            state["sentiment_source"] = result.source
            state["sentiment_last_updated"] = result.timestamp
            state["sentiment_texts_analyzed"] = result.texts_analyzed
            self.state_path.write_text(
                json.dumps(state, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except Exception as exc:
            logger.error("Error escribiendo sentiment en state: %s", exc)

    def _build_cached_result(self) -> SentimentResult:
        """Construye un resultado con el último valor en caché."""
        return SentimentResult(
            score=self._last_score,
            source="cache",
            texts_analyzed=0,
            positive_pct=0.0,
            negative_pct=0.0,
            neutral_pct=1.0,
            timestamp=datetime.now(timezone.utc).isoformat(),
        )
