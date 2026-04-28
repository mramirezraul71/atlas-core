"""
lotto_quant.data.scratchsmarter_scraper
=======================================

Secondary validation source for NCEL scratch-off data.

Currently targets `scratchodds.com/north-carolina/scratch-off-remaining-prizes`
(formerly known as ScratchSmarter).  The page aggregates the same NCEL data
plus a "real odds" estimate.  We use it ONLY to cross-check NCELScraper output
and flag discrepancies.
"""

from __future__ import annotations

import logging
import re
from datetime import datetime
from typing import List, Optional

import httpx
from bs4 import BeautifulSoup

from .. import config
from ..models.ev_calculator import PrizeTier, ScratchOffGame

logger = logging.getLogger(__name__)


class ScratchOddsScraper:
    """Lightweight scraper for scratchodds.com (validation only)."""

    BASE_URL = config.SCRATCHODDS_URL

    def __init__(self, timeout_s: int = config.REQUEST_TIMEOUT_S):
        self.timeout_s = timeout_s
        self._headers = {"User-Agent": config.USER_AGENT}

    async def fetch_all_games(self) -> List[ScratchOffGame]:
        async with httpx.AsyncClient(
            timeout=self.timeout_s, headers=self._headers, follow_redirects=True
        ) as client:
            try:
                r = await client.get(self.BASE_URL)
                r.raise_for_status()
            except httpx.HTTPError as e:
                logger.warning("ScratchOdds fetch failed: %s", e)
                return []
            return self._parse(r.text)

    def _parse(self, html: str) -> List[ScratchOffGame]:
        soup = BeautifulSoup(html, "lxml" if _has_lxml() else "html.parser")
        games: List[ScratchOffGame] = []
        for row in soup.select("table tr"):
            cells = row.find_all("td")
            if len(cells) < 4:
                continue
            try:
                name = cells[0].get_text(strip=True)
                price_match = re.search(r"\$([\d.]+)", cells[1].get_text())
                if not price_match:
                    continue
                price = float(price_match.group(1))
                top_prize_match = re.search(r"\$([\d,]+)", cells[2].get_text())
                if not top_prize_match:
                    continue
                top_prize = float(top_prize_match.group(1).replace(",", ""))
                top_remaining = re.sub(r"[^\d]", "", cells[3].get_text())
                if not top_remaining:
                    continue
                tier = PrizeTier(
                    value=top_prize,
                    total_prizes=int(top_remaining) * 10,  # rough estimate; aggregator only shows top
                    remaining_prizes=int(top_remaining),
                )
                games.append(
                    ScratchOffGame(
                        game_id=_slug(name),
                        name=name,
                        ticket_price=price,
                        prize_tiers=[tier],
                        snapshot_ts=datetime.utcnow().isoformat(),
                    )
                )
            except Exception as e:  # pragma: no cover
                logger.debug("ScratchOdds row parse failed: %s", e)
        return games

    @staticmethod
    def reconcile(
        primary: List[ScratchOffGame], secondary: List[ScratchOffGame]
    ) -> dict:
        """
        Compare two scrapes and return a discrepancy report.
        """
        primary_idx = {g.name.lower(): g for g in primary}
        secondary_idx = {g.name.lower(): g for g in secondary}
        discrepancies = []
        common = primary_idx.keys() & secondary_idx.keys()
        for name in common:
            p = primary_idx[name]
            s = secondary_idx[name]
            if abs(p.ticket_price - s.ticket_price) > 0.01:
                discrepancies.append(
                    {
                        "game": name,
                        "field": "ticket_price",
                        "primary": p.ticket_price,
                        "secondary": s.ticket_price,
                    }
                )
        return {
            "n_primary": len(primary),
            "n_secondary": len(secondary),
            "n_common": len(common),
            "discrepancies": discrepancies,
        }


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "unknown"


def _has_lxml() -> bool:
    try:
        import lxml  # noqa: F401
        return True
    except ImportError:
        return False
