"""
Proveedor yfinance  –  GRATUITO, solo para desarrollo/backtest offline.
NO usar en producción: sin garantía de calidad de datos.
Útil para prototipar estrategias antes de conectar un proveedor de pago.
"""
from __future__ import annotations
import math
from datetime import date, datetime
from typing import Optional

from .base import OptionsDataProvider
from ..models.option_contract import (
    OptionContract, OptionsChain, OptionType, Greeks
)


class YFinanceProvider(OptionsDataProvider):
    """Proveedor basado en yfinance (dev/backtest only)."""

    def __init__(self):
        try:
            import yfinance as yf
            self._yf = yf
        except ImportError:
            raise ImportError("Instala yfinance: pip install yfinance")

    def get_quote(self, symbol: str) -> float:
        ticker = self._yf.Ticker(symbol)
        info   = ticker.fast_info
        return float(info.get("lastPrice", info.get("regularMarketPrice", 0)))

    def get_expirations(self, symbol: str) -> list[date]:
        ticker = self._yf.Ticker(symbol)
        return [datetime.strptime(e, "%Y-%m-%d").date() for e in ticker.options]

    def get_chain(
        self,
        symbol: str,
        expiration: date,
        option_type: Optional[str] = None,
    ) -> OptionsChain:
        ticker  = self._yf.Ticker(symbol)
        exp_str = expiration.strftime("%Y-%m-%d")
        opt     = ticker.option_chain(exp_str)
        spot    = self.get_quote(symbol)

        contracts: list[OptionContract] = []
        frames = []
        if option_type in (None, "call"):
            frames.append(("call", opt.calls))
        if option_type in (None, "put"):
            frames.append(("put", opt.puts))

        def _safe_int(val, default: int = 0) -> int:
            try:
                x = float(val)
            except (TypeError, ValueError):
                return default
            if math.isnan(x) or math.isinf(x):
                return default
            return int(x)

        def _safe_float(val, default: float = 0.0) -> float:
            try:
                x = float(val)
            except (TypeError, ValueError):
                return default
            if math.isnan(x) or math.isinf(x):
                return default
            return x

        def _row_delta(row) -> float:
            for key in ("delta", "Delta"):
                if key not in row.index:
                    continue
                v = row.get(key)
                if v is None:
                    continue
                d = _safe_float(v, 0.0)
                if abs(d) > 1e-12:
                    return d
            return 0.0

        for ot, df in frames:
            for _, row in df.iterrows():
                strike = _safe_float(row.get("strike"), 0.0)
                if strike <= 0:
                    continue
                last_price = _safe_float(row.get("lastPrice", 0), 0.0)
                bid = _safe_float(row.get("bid", 0), 0.0)
                ask = _safe_float(row.get("ask", 0), 0.0)
                if last_price <= 0 and bid <= 0 and ask <= 0:
                    continue
                iv = _safe_float(row.get("impliedVolatility", 0), 0.0)
                contracts.append(OptionContract(
                    symbol          = symbol,
                    option_type     = OptionType.CALL if ot == "call" else OptionType.PUT,
                    strike          = strike,
                    expiration      = expiration,
                    last_price      = last_price,
                    bid             = bid,
                    ask             = ask,
                    volume          = _safe_int(row.get("volume", 0)),
                    open_interest   = _safe_int(row.get("openInterest", 0)),
                    contract_symbol = str(row.get("contractSymbol", "") or ""),
                    greeks          = Greeks(
                        delta=_row_delta(row),
                        iv=iv,
                    ),
                ))

        return OptionsChain(
            symbol     = symbol,
            expiration = expiration,
            spot_price = spot,
            contracts  = contracts,
        )
