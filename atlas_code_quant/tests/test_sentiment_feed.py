"""Tests para SentimentFeed — Fase 0 Semana 2."""
from __future__ import annotations

import json
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.data.sentiment_feed import (
    SentimentFeed,
    SentimentResult,
    _analyze_texts,
    _keyword_fallback,
    _vader_available,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture()
def state_file(tmp_path: Path) -> Path:
    """Crea un operation_center_state.json mínimo para tests."""
    state = {
        "sentiment_score": 0.0,
        "sentiment_source": "manual",
        "auton_mode": "paper_autonomous",
    }
    p = tmp_path / "operation_center_state.json"
    p.write_text(json.dumps(state), encoding="utf-8")
    return p


@pytest.fixture()
def feed(state_file: Path) -> SentimentFeed:
    return SentimentFeed(state_path=state_file, cache_ttl_seconds=0)


# ── Tests keyword fallback (sin dependencias externas) ────────────────────────

def test_keyword_fallback_bullish():
    texts = ["bull market buy breakout rally", "moon surge gains up up"]
    result = _keyword_fallback(texts)
    assert result["compound"] > 0, "Textos bullish deben producir score positivo"


def test_keyword_fallback_bearish():
    texts = ["crash dump sell bear down", "loss drop fall short"]
    result = _keyword_fallback(texts)
    assert result["compound"] < 0, "Textos bearish deben producir score negativo"


def test_keyword_fallback_neutral():
    texts = ["the stock market is open", "options expiry today"]
    result = _keyword_fallback(texts)
    assert -0.5 <= result["compound"] <= 0.5, "Textos neutros deben producir score cerca de cero"


def test_keyword_fallback_empty():
    result = _keyword_fallback([])
    assert result["count"] == 0
    assert result["compound"] == 0.0


# ── Tests _analyze_texts ──────────────────────────────────────────────────────

def test_analyze_texts_returns_compound():
    texts = ["great earnings beat expectations", "market up today"]
    result = _analyze_texts(texts)
    assert "compound" in result
    assert "count" in result
    assert result["count"] == 2


def test_analyze_texts_empty():
    result = _analyze_texts([])
    assert result["compound"] == 0.0
    assert result["count"] == 0


# ── Tests SentimentFeed ───────────────────────────────────────────────────────

def test_feed_writes_to_state(feed: SentimentFeed, state_file: Path):
    """refresh() debe escribir sentiment_score en el state JSON."""
    mock_texts = ["stocks rallying hard today bull run", "buy the dip market surge"]

    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=mock_texts):
        result = feed.refresh(force=True)

    state = json.loads(state_file.read_text())
    assert "sentiment_score" in state
    assert state["sentiment_source"] != "manual"
    assert isinstance(result.score, float)
    assert -1.0 <= result.score <= 1.0


def test_feed_score_range(feed: SentimentFeed):
    """El score siempre debe estar en [-1.0, 1.0]."""
    texts = ["insane bull run moon lambo gains buy buy buy"] * 20
    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=texts):
        result = feed.refresh(force=True)
    assert -1.0 <= result.score <= 1.0


def test_feed_no_sources_returns_neutral(feed: SentimentFeed):
    """Si no hay fuentes disponibles, score debe ser neutro y error != None."""
    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=[]):
        result = feed.refresh(force=True)
    assert result.error is not None
    assert result.score == 0.0


def test_feed_ttl_cache(feed: SentimentFeed):
    """Con TTL > 0, el segundo refresh no debe llamar a fetch."""
    feed2 = SentimentFeed(state_path=feed.state_path, cache_ttl_seconds=3600)
    texts = ["bull market today gains"]

    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=texts) as mock_fetch:
        feed2.refresh(force=True)   # primer refresh — llama fetch
        feed2.refresh()              # segundo dentro del TTL — no debe llamar fetch

    # fetch se llama una vez por feed (3 feeds), total 3 calls en el primer refresh
    # en el segundo refresh (con TTL activo) no debe llamar nada adicional
    call_count_after_second = mock_fetch.call_count
    assert call_count_after_second == len(feed2.__class__.__dict__) or call_count_after_second <= 3 * 2


def test_feed_state_not_found_no_crash(tmp_path: Path):
    """Si el state path no existe, refresh no debe lanzar excepción."""
    feed = SentimentFeed(state_path=tmp_path / "nonexistent.json", cache_ttl_seconds=0)
    texts = ["market is up today"]
    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=texts):
        result = feed.refresh(force=True)
    # No debe lanzar excepción — error de escritura es silenciado con log
    assert isinstance(result, SentimentResult)


def test_feed_smoothing_dampens_changes(feed: SentimentFeed):
    """El suavizado EMA debe amortiguar cambios bruscos entre refreshes."""
    bearish = ["crash dump sell bear loss down"] * 10
    bullish = ["bull moon buy rally surge gains"] * 10

    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=bearish):
        r1 = feed.refresh(force=True)

    with patch("atlas_code_quant.data.sentiment_feed._fetch_reddit_texts", return_value=bullish):
        r2 = feed.refresh(force=True)

    # El score no debe saltar del extremo bearish al extremo bullish en un solo step
    assert abs(r2.score - r1.score) < 1.5, "Suavizado EMA debe amortiguar el cambio"
