from __future__ import annotations

import asyncio
import json
from datetime import datetime, timezone
from typing import AsyncIterator, Optional

import httpx

from .config import RadarSettings, get_settings
from .scanner import MarketEvent
from .utils.logger import get_logger


class PolymarketScanner:
    """Discovery scanner de Polymarket (REST-first, realtime-friendly)."""

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("polymarket_scanner", self.settings.log_dir, self.settings.log_level)
        self._seen: set[str] = set()
        self.reconnect_count: int = 0

    async def stream(self) -> AsyncIterator[MarketEvent]:
        if not self.settings.polymarket_enabled:
            return
        while True:
            try:
                async for ev in self.discover():
                    yield ev
            except Exception as exc:
                self.reconnect_count += 1
                self.log.warning("Polymarket discover falló: %s", exc)
                err_delay = min(120, 5 * (2 ** min(self.reconnect_count, 5)))
                await asyncio.sleep(err_delay)
                continue
            await asyncio.sleep(max(5, int(self.settings.poll_interval_seconds)))

    async def discover(self) -> AsyncIterator[MarketEvent]:
        url = f"{self.settings.polymarket_gamma_url.rstrip('/')}/markets"
        params = {"active": "true", "closed": "false", "limit": 200}
        async with httpx.AsyncClient(timeout=20) as client:
            r = await client.get(url, params=params)
            r.raise_for_status()
            markets = r.json() or []
        now = datetime.now(timezone.utc)
        for market in markets:
            mid = self._to_float(market.get("lastTradePrice"))
            if mid is None:
                outcome_prices = market.get("outcomePrices")
                if isinstance(outcome_prices, str):
                    try:
                        parsed = json.loads(outcome_prices)
                        if isinstance(parsed, list) and parsed:
                            mid = self._to_float(parsed[0])
                    except Exception:
                        mid = None
            if mid is None:
                continue
            ticker = f"POLY:{market.get('id') or market.get('conditionId')}"
            yes_tok, no_tok = None, None
            cti = market.get("clobTokenIds")
            if cti:
                try:
                    arr = json.loads(cti) if isinstance(cti, str) else cti
                    if isinstance(arr, list) and len(arr) >= 2:
                        yes_tok, no_tok = str(arr[0]), str(arr[1])
                except Exception:
                    yes_tok, no_tok = None, None
            payload = {
                "source": "polymarket",
                "title": market.get("question") or market.get("description") or ticker,
                "yes_bid": max(0.01, min(0.99, mid - 0.01)),
                "yes_ask": max(0.01, min(0.99, mid + 0.01)),
                "yes_bid_size": int(float(market.get("liquidityNum") or 50)),
                "yes_ask_size": int(float(market.get("liquidityNum") or 50)),
                "close_time": market.get("endDate"),
                "market_id": market.get("id"),
                "yes_clob_token_id": yes_tok,
                "no_clob_token_id": no_tok,
            }
            if ticker not in self._seen:
                self._seen.add(ticker)
                yield MarketEvent(kind="new_market", market_ticker=ticker, payload=payload, ts=now)
            yield MarketEvent(kind="ticker", market_ticker=ticker, payload=payload, ts=now)

    @staticmethod
    def _to_float(v: object) -> Optional[float]:
        try:
            return float(v)  # type: ignore[arg-type]
        except Exception:
            return None

