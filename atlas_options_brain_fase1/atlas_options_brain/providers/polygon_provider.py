"""
Proveedor Polygon.io  –  $79/mes (Developer) para opciones.
Ideal si Atlas ya usa Polygon para equities/crypto.

Docs: https://polygon.io/docs/options
"""
from __future__ import annotations
import os
import requests
from datetime import date, datetime
from typing import Optional

from .base import OptionsDataProvider
from ..models.option_contract import (
    OptionContract, OptionsChain, OptionType, Greeks
)

POLYGON_BASE = "https://api.polygon.io"


def _parse_date(s: str) -> date:
    return datetime.strptime(s[:10], "%Y-%m-%d").date()


class PolygonProvider(OptionsDataProvider):
    """
    Proveedor Polygon.io.

    Uso:
        provider = PolygonProvider(api_key=os.getenv("POLYGON_API_KEY"))
        chain    = provider.get_chain("SPY", date(2025, 6, 20))
    """

    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.getenv("POLYGON_API_KEY", "")

    def _get(self, endpoint: str, params: dict = {}) -> dict:
        params["apiKey"] = self.api_key
        resp = requests.get(f"{POLYGON_BASE}{endpoint}", params=params, timeout=10)
        resp.raise_for_status()
        return resp.json()

    def get_quote(self, symbol: str) -> float:
        data = self._get(f"/v2/last/trade/{symbol}")
        return float(data.get("results", {}).get("p", 0))

    def get_expirations(self, symbol: str) -> list[date]:
        """Polygon no tiene endpoint de expiraciones; se derivan de la cadena."""
        # Workaround: listar contratos únicos y extraer fechas
        data = self._get(
            "/v3/reference/options/contracts",
            {"underlying_ticker": symbol, "limit": 250, "expired": "false"},
        )
        results = data.get("results", [])
        exps = sorted({_parse_date(r["expiration_date"]) for r in results})
        return exps

    def get_chain(
        self,
        symbol: str,
        expiration: date,
        option_type: Optional[str] = None,
    ) -> OptionsChain:
        params: dict = {
            "underlying_ticker":    symbol,
            "expiration_date":      expiration.strftime("%Y-%m-%d"),
            "limit":                250,
            "contract_type":        option_type or "",
        }
        if not option_type:
            del params["contract_type"]

        data     = self._get("/v3/snapshot/options/" + symbol, params)
        raw_list = data.get("results", [])

        spot = self.get_quote(symbol)
        contracts = []
        for r in raw_list:
            det = r.get("details", {})
            day = r.get("day", {})
            gk  = r.get("greeks", {})
            contracts.append(OptionContract(
                symbol          = symbol,
                option_type     = OptionType.CALL if det.get("contract_type") == "call" else OptionType.PUT,
                strike          = float(det.get("strike_price", 0)),
                expiration      = _parse_date(det.get("expiration_date", str(expiration))),
                last_price      = float(day.get("close") or r.get("last_quote", {}).get("ask", 0)),
                bid             = float(r.get("last_quote", {}).get("bid", 0)),
                ask             = float(r.get("last_quote", {}).get("ask", 0)),
                volume          = int(day.get("volume", 0)),
                open_interest   = int(r.get("open_interest", 0)),
                contract_symbol = det.get("ticker", ""),
                greeks          = Greeks(
                    delta = float(gk.get("delta", 0)),
                    gamma = float(gk.get("gamma", 0)),
                    theta = float(gk.get("theta", 0)),
                    vega  = float(gk.get("vega",  0)),
                    iv    = float(r.get("implied_volatility", 0)),
                ),
            ))

        return OptionsChain(
            symbol     = symbol,
            expiration = expiration,
            spot_price = spot,
            contracts  = contracts,
        )
