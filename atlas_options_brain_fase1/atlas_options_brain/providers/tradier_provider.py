"""
Proveedor Tradier  –  $10/mes + ejecución como broker.
RECOMENDADO para Atlas porque unifica datos + órdenes en un solo API.

.. legacy:: F4 PHASE1
    Cliente de **datos** Tradier perteneciente al stack legacy/PHASE1
    (``atlas_options_brain_fase1``). NO es la ruta canónica de envio de
    órdenes. La lógica de routing/ejecución canónica vive en
    ``atlas_code_quant.execution.tradier_execution``.

    F4 sólo etiqueta este módulo; no cambia firmas ni lógica.
    Ver ``docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md``.

Docs: https://documentation.tradier.com/brokerage-api
Endpoint sandbox: https://sandbox.tradier.com/v1
Endpoint live:    https://api.tradier.com/v1
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


TRADIER_LIVE    = "https://api.tradier.com/v1"
TRADIER_SANDBOX = "https://sandbox.tradier.com/v1"


def _parse_date(s: str) -> date:
    return datetime.strptime(s, "%Y-%m-%d").date()


def _parse_contract(raw: dict, symbol: str, expiration: date) -> OptionContract:
    greeks_raw = raw.get("greeks") or {}
    return OptionContract(
        symbol          = symbol,
        option_type     = OptionType.CALL if raw["option_type"] == "call" else OptionType.PUT,
        strike          = float(raw["strike"]),
        expiration      = expiration,
        last_price      = float(raw.get("last") or 0),
        bid             = float(raw.get("bid") or 0),
        ask             = float(raw.get("ask") or 0),
        volume          = int(raw.get("volume") or 0),
        open_interest   = int(raw.get("open_interest") or 0),
        contract_symbol = raw.get("symbol", ""),
        greeks          = Greeks(
            delta = float(greeks_raw.get("delta") or 0),
            gamma = float(greeks_raw.get("gamma") or 0),
            theta = float(greeks_raw.get("theta") or 0),
            vega  = float(greeks_raw.get("vega")  or 0),
            rho   = float(greeks_raw.get("rho")   or 0),
            iv    = float(raw.get("smv_vol") or greeks_raw.get("mid_iv") or 0),
        ),
    )


class TradierProvider(OptionsDataProvider):
    """
    Proveedor Tradier.  Soporta sandbox y live.

    Uso:
        provider = TradierProvider(token=os.getenv("TRADIER_TOKEN"), sandbox=True)
        chain    = provider.get_chain("SPY", date(2025, 6, 20))
    """

    def __init__(
        self,
        token: Optional[str] = None,
        sandbox: bool = True,
    ):
        self.token   = token or os.getenv("TRADIER_TOKEN", "")
        self.base    = TRADIER_SANDBOX if sandbox else TRADIER_LIVE
        self.headers = {
            "Authorization": f"Bearer {self.token}",
            "Accept":        "application/json",
        }

    def _get(self, endpoint: str, params: dict = {}) -> dict:
        resp = requests.get(
            f"{self.base}{endpoint}",
            headers = self.headers,
            params  = params,
            timeout = 10,
        )
        resp.raise_for_status()
        return resp.json()

    def get_quote(self, symbol: str) -> float:
        data = self._get("/markets/quotes", {"symbols": symbol, "greeks": "false"})
        quote = data["quotes"]["quote"]
        return float(quote.get("last") or quote.get("ask") or 0)

    def get_expirations(self, symbol: str) -> list[date]:
        data = self._get("/markets/options/expirations", {"symbol": symbol, "includeAllRoots": "true"})
        exps = data.get("expirations", {}).get("date", [])
        if isinstance(exps, str):
            exps = [exps]
        return [_parse_date(e) for e in exps]

    def get_chain(
        self,
        symbol: str,
        expiration: date,
        option_type: Optional[str] = None,
    ) -> OptionsChain:
        params: dict = {
            "symbol":     symbol,
            "expiration": expiration.strftime("%Y-%m-%d"),
            "greeks":     "true",
        }
        if option_type:
            params["optionType"] = option_type

        data     = self._get("/markets/options/chains", params)
        raw_list = data.get("options", {}).get("option", [])
        if isinstance(raw_list, dict):
            raw_list = [raw_list]

        spot = self.get_quote(symbol)
        contracts = [_parse_contract(r, symbol, expiration) for r in raw_list]

        return OptionsChain(
            symbol     = symbol,
            expiration = expiration,
            spot_price = spot,
            contracts  = contracts,
        )
