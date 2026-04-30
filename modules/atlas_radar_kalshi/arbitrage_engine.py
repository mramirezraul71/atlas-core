from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .normalization import canonical_market_key


@dataclass
class VenueQuote:
    ticker: str
    source: str  # kalshi | polymarket
    title: str
    close_time: str | None
    yes_mid: float
    yes_ask: Optional[float] = None
    no_ask: Optional[float] = None


class ArbitrageEngine:
    """Cross-venue (Kalshi vs Polymarket) e intra-venue binario (caja YES+NO)."""

    def __init__(self, min_profit: float = 0.02) -> None:
        self.min_profit = min_profit
        self._quotes: dict[str, dict[str, VenueQuote]] = {}

    def upsert_quote(
        self,
        *,
        ticker: str,
        source: str,
        title: str,
        close_time: str | None,
        yes_mid: float,
        yes_ask: float | None = None,
        no_ask: float | None = None,
    ) -> None:
        key = canonical_market_key(title, close_time)
        per_venue = self._quotes.setdefault(key, {})
        per_venue[source] = VenueQuote(
            ticker=ticker,
            source=source,
            title=title,
            close_time=close_time,
            yes_mid=max(0.01, min(0.99, float(yes_mid))),
            yes_ask=yes_ask,
            no_ask=no_ask,
        )

    @staticmethod
    def _intra_box(
        v: VenueQuote, canonical_key: str, label: str, min_profit: float
    ) -> dict | None:
        if v.yes_ask is None or v.no_ask is None:
            return None
        cost = float(v.yes_ask) + float(v.no_ask)
        profit = 1.0 - cost
        if profit < min_profit:
            return None
        return {
            "canonical_key": f"{canonical_key}|{label}_BOX",
            "strategy": f"INTRA_{label}_BOX",
            "profit_estimate": round(profit, 6),
            "venue": label.lower(),
            "leg_ticker": v.ticker,
            "yes_ask": round(v.yes_ask, 4),
            "no_ask": round(v.no_ask, 4),
        }

    def detect(self, top_n: int = 20) -> list[dict]:
        found: list[dict] = []
        for ckey, q in self._quotes.items():
            k = q.get("kalshi")
            p = q.get("polymarket")
            if k:
                ib = self._intra_box(k, ckey, "KALSHI", self.min_profit)
                if ib:
                    found.append(ib)
            if p:
                ibp = self._intra_box(p, ckey, "POLY", self.min_profit)
                if ibp:
                    found.append(ibp)
            if not k or not p:
                continue
            cost_a = k.yes_mid + (1.0 - p.yes_mid)
            profit_a = 1.0 - cost_a
            cost_b = (1.0 - k.yes_mid) + p.yes_mid
            profit_b = 1.0 - cost_b
            best_profit = max(profit_a, profit_b)
            if best_profit < self.min_profit:
                continue
            strategy = "YES_KALSHI_NO_POLY" if profit_a >= profit_b else "NO_KALSHI_YES_POLY"
            found.append(
                {
                    "canonical_key": ckey,
                    "strategy": strategy,
                    "profit_estimate": round(best_profit, 6),
                    "kalshi_ticker": k.ticker,
                    "polymarket_ticker": p.ticker,
                    "kalshi_yes": round(k.yes_mid, 4),
                    "polymarket_yes": round(p.yes_mid, 4),
                }
            )
        found.sort(key=lambda x: x.get("profit_estimate", 0) or 0, reverse=True)
        return found[:top_n]

    @staticmethod
    def build_execution_intents(opportunity: dict, contracts: int) -> list[dict]:
        c = max(1, int(contracts))
        strat = str(opportunity.get("strategy", ""))
        if strat in ("INTRA_KALSHI_BOX", "INTRA_POLY_BOX"):
            t = str(opportunity.get("leg_ticker", ""))
            y = float(opportunity.get("yes_ask", 0.5))
            n = float(opportunity.get("no_ask", 0.5))
            return [
                {
                    "ticker": t,
                    "side": "YES",
                    "price_cents": max(1, min(99, int(round(y * 100)))),
                    "contracts": c,
                },
                {
                    "ticker": t,
                    "side": "NO",
                    "price_cents": max(1, min(99, int(round(n * 100)))),
                    "contracts": c,
                },
            ]
        k_ticker = str(opportunity.get("kalshi_ticker", ""))
        p_ticker = str(opportunity.get("polymarket_ticker", ""))
        k_yes = float(opportunity.get("kalshi_yes", 0.5))
        p_yes = float(opportunity.get("polymarket_yes", 0.5))
        if strat == "YES_KALSHI_NO_POLY":
            return [
                {
                    "ticker": k_ticker,
                    "side": "YES",
                    "price_cents": max(1, min(99, int(round(k_yes * 100)))),
                    "contracts": c,
                },
                {
                    "ticker": p_ticker,
                    "side": "NO",
                    "price_cents": max(1, min(99, int(round((1 - p_yes) * 100)))),
                    "contracts": c,
                },
            ]
        if strat == "NO_KALSHI_YES_POLY":
            return [
                {
                    "ticker": k_ticker,
                    "side": "NO",
                    "price_cents": max(1, min(99, int(round((1 - k_yes) * 100)))),
                    "contracts": c,
                },
                {
                    "ticker": p_ticker,
                    "side": "YES",
                    "price_cents": max(1, min(99, int(round(p_yes * 100)))),
                    "contracts": c,
                },
            ]
        return []
