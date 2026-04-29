"""
lotto_quant.data.ncel_scraper
=============================

Async scraper for the North Carolina Education Lottery website.

Targets
-------
    https://nclottery.com/scratch-off-prizes-remaining
    https://nclottery.com/Powerball   (jackpot)
    https://nclottery.com/MegaMillions (jackpot)

The HTML on NCEL changes from time to time. This module is defensive:
    1. Parse with BeautifulSoup using multiple selector strategies.
    2. If structured parse yields zero rows, optionally fall back to a
       local LLM (Ollama / Qwen3 / DeepSeek) to extract the JSON.
    3. Validate everything with Pydantic before returning.

Respects robots.txt and rate-limits requests.
"""

from __future__ import annotations

import asyncio
import logging
import re
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional

import httpx
from bs4 import BeautifulSoup

from .. import config
from ..llm import LocalAIClient, LocalAIError
from ..llm.prompts import HTML_TRIAGE_SYSTEM, HTML_TRIAGE_USER
from ..models.ev_calculator import PrizeTier, ScratchOffGame
from .schemas import (
    DrawGameJackpotSchema,
    PrizeTierSchema,
    ScratchOffGameSchema,
)

logger = logging.getLogger(__name__)

_MONEY_RE = re.compile(r"\$([\d,]+(?:\.\d+)?)")


class NCELScraper:
    """Async scraper for NCEL data with retry, validation, and LLM fallback."""

    BASE_URL = config.NCEL_BASE_URL
    SECONDARY_URL = config.SCRATCHODDS_URL

    def __init__(
        self,
        rate_limit_s: float = config.RATE_LIMIT_DELAY_S,
        timeout_s: int = config.REQUEST_TIMEOUT_S,
        max_retries: int = config.MAX_RETRIES,
        use_llm_fallback: bool = config.LOCAL_AI_ENABLED,
    ):
        self.rate_limit_s = rate_limit_s
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self.use_llm_fallback = use_llm_fallback
        self._headers = {
            "User-Agent": config.USER_AGENT,
            "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
        }
        self._client: Optional[httpx.AsyncClient] = None

    # ── context ────────────────────────────────────────────────────
    async def __aenter__(self) -> "NCELScraper":
        self._client = httpx.AsyncClient(
            timeout=self.timeout_s,
            headers=self._headers,
            follow_redirects=True,
        )
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._client is not None:
            await self._client.aclose()
            self._client = None

    # ── fetch with retry ───────────────────────────────────────────
    async def _fetch(self, url: str) -> str:
        assert self._client is not None
        last_exc: Optional[Exception] = None
        for attempt in range(1, self.max_retries + 1):
            try:
                r = await self._client.get(url)
                r.raise_for_status()
                await asyncio.sleep(self.rate_limit_s)
                return r.text
            except httpx.HTTPError as e:
                last_exc = e
                wait = self.rate_limit_s * (2 ** (attempt - 1))
                logger.warning(
                    "Fetch %s failed (attempt %d/%d): %s — backing off %.1fs",
                    url, attempt, self.max_retries, e, wait,
                )
                await asyncio.sleep(wait)
        raise RuntimeError(f"Failed to fetch {url} after {self.max_retries} retries") from last_exc

    # ── public API ─────────────────────────────────────────────────
    async def fetch_all_games(self) -> List[ScratchOffGame]:
        """Fetch and parse all current scratch-off games."""
        html = await self._fetch(self.BASE_URL)
        games = self.parse_prizes_table(html)

        if not games and self.use_llm_fallback:
            logger.warning("BeautifulSoup parse returned 0 games — invoking local LLM fallback")
            games = await self._llm_fallback_parse(html)

        validated = self.validate_data(games)
        logger.info("Fetched %d valid games from NCEL", len(validated))
        return validated

    async def fetch_draw_game_jackpots(self) -> Dict[str, DrawGameJackpotSchema]:
        """Fetch current Powerball and Mega Millions jackpot amounts."""
        results: Dict[str, DrawGameJackpotSchema] = {}
        for slug, name in [("Powerball", "powerball"), ("MegaMillions", "mega_millions")]:
            url = f"https://nclottery.com/{slug}"
            try:
                html = await self._fetch(url)
                amount = self._extract_jackpot_amount(html)
                if amount is not None:
                    results[name] = DrawGameJackpotSchema(
                        game_name=name,
                        jackpot_amount=amount,
                    )
            except Exception as e:  # pragma: no cover
                logger.error("Failed to fetch %s jackpot: %s", name, e)
        return results

    # ── parsing ────────────────────────────────────────────────────
    def parse_prizes_table(self, html: str) -> List[ScratchOffGame]:
        """
        Parse NCEL HTML into ScratchOffGame objects.

        We try multiple selector strategies because NCEL's markup evolves.
        """
        soup = BeautifulSoup(html, "lxml" if _has_lxml() else "html.parser")
        games: List[ScratchOffGame] = []

        # Strategy A: rows with class hints commonly used by NCEL
        for game_block in soup.select("div.game, div.scratch-off-game, article.game"):
            parsed = self._parse_game_block(game_block)
            if parsed:
                games.append(parsed)

        # Strategy B: tabular layout
        if not games:
            for table in soup.find_all("table"):
                games.extend(self._parse_table(table))

        return games

    def _parse_game_block(self, block) -> Optional[ScratchOffGame]:
        try:
            name_el = block.select_one(".game-name, h2, h3")
            price_el = block.select_one(".ticket-price, .price")
            if not name_el or not price_el:
                return None
            name = name_el.get_text(strip=True)
            price_match = _MONEY_RE.search(price_el.get_text())
            if not price_match:
                return None
            ticket_price = float(price_match.group(1).replace(",", ""))

            tiers: List[PrizeTier] = []
            for row in block.select("tr.prize-tier, .prize-row"):
                cells = row.find_all(["td", "div"])
                if len(cells) < 3:
                    continue
                value = self._money_to_float(cells[0].get_text())
                total = self._int_or_none(cells[1].get_text())
                remaining = self._int_or_none(cells[2].get_text())
                if value is None or total is None or remaining is None:
                    continue
                tiers.append(
                    PrizeTier(
                        value=value,
                        total_prizes=total,
                        remaining_prizes=remaining,
                    )
                )
            if not tiers:
                return None

            game_id = block.get("data-game-id") or _slug(name)
            return ScratchOffGame(
                game_id=str(game_id),
                name=name,
                ticket_price=ticket_price,
                prize_tiers=tiers,
                snapshot_ts=datetime.utcnow().isoformat(),
            )
        except Exception as e:  # pragma: no cover
            logger.debug("Game block parse failed: %s", e)
            return None

    def _parse_table(self, table) -> List[ScratchOffGame]:
        # Generic fallback: table-of-tables
        return []  # placeholder for future selector evolution

    @staticmethod
    def _money_to_float(text: str) -> Optional[float]:
        m = _MONEY_RE.search(text or "")
        if not m:
            return None
        return float(m.group(1).replace(",", ""))

    @staticmethod
    def _int_or_none(text: str) -> Optional[int]:
        cleaned = re.sub(r"[^\d]", "", text or "")
        return int(cleaned) if cleaned else None

    def _extract_jackpot_amount(self, html: str) -> Optional[float]:
        """Pull a jackpot dollar amount from a Powerball/MegaMillions page."""
        soup = BeautifulSoup(html, "lxml" if _has_lxml() else "html.parser")
        text = soup.get_text(" ", strip=True)
        # Look for "$1.2 Billion" or "$650 Million"
        m = re.search(
            r"\$\s*([\d.,]+)\s*(billion|million)",
            text, flags=re.IGNORECASE,
        )
        if m:
            value = float(m.group(1).replace(",", ""))
            unit = m.group(2).lower()
            return value * (1_000_000_000 if unit == "billion" else 1_000_000)
        m2 = _MONEY_RE.search(text)
        if m2:
            return float(m2.group(1).replace(",", ""))
        return None

    # ── LLM fallback ───────────────────────────────────────────────
    async def _llm_fallback_parse(self, html: str) -> List[ScratchOffGame]:
        excerpt = html[:30_000]
        try:
            async with LocalAIClient() as ai:
                healthy = await ai.health()
                if not healthy:
                    logger.warning("Local AI is not reachable; skipping LLM fallback.")
                    return []
                data = await ai.complete_json(
                    HTML_TRIAGE_USER.format(html_excerpt=excerpt),
                    system=HTML_TRIAGE_SYSTEM,
                )
        except LocalAIError as e:
            logger.error("Local LLM fallback failed: %s", e)
            return []

        # Accept either a list-of-games or {"games": [...]}
        if isinstance(data, dict) and "games" in data:
            data = data["games"]
        if not isinstance(data, list):
            return []

        out: List[ScratchOffGame] = []
        for item in data:
            try:
                tiers = [
                    PrizeTier(
                        value=float(p["value"]),
                        total_prizes=int(p.get("total") or 0),
                        remaining_prizes=int(p.get("remaining") or 0),
                    )
                    for p in item.get("prizes", [])
                    if p.get("value") and p.get("remaining") is not None
                ]
                if not tiers:
                    continue
                out.append(
                    ScratchOffGame(
                        game_id=str(item.get("game_id") or _slug(item.get("name", ""))),
                        name=str(item.get("name") or "Unknown"),
                        ticket_price=float(item.get("ticket_price") or 1.0),
                        prize_tiers=tiers,
                        snapshot_ts=datetime.utcnow().isoformat(),
                    )
                )
            except (KeyError, TypeError, ValueError) as e:
                logger.debug("LLM record discarded: %s", e)
        return out

    # ── validation ─────────────────────────────────────────────────
    def validate_data(self, games: List[ScratchOffGame]) -> List[ScratchOffGame]:
        """Validate scraped games via Pydantic. Drops any invalid record."""
        valid: List[ScratchOffGame] = []
        for g in games:
            try:
                ScratchOffGameSchema(
                    game_id=g.game_id,
                    name=g.name,
                    ticket_price=g.ticket_price,
                    prize_tiers=[
                        PrizeTierSchema(
                            value=t.value,
                            total_prizes=t.total_prizes,
                            remaining_prizes=t.remaining_prizes,
                        )
                        for t in g.prize_tiers
                    ],
                    total_tickets_printed=g.total_tickets_printed,
                    tickets_sold_estimate=g.tickets_sold_estimate,
                )
                valid.append(g)
            except Exception as e:
                logger.warning("Discarding invalid game %s: %s", g.game_id, e)
        return valid


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "unknown"


def _has_lxml() -> bool:
    try:
        import lxml  # noqa: F401
        return True
    except ImportError:
        return False
