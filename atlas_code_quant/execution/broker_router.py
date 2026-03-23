"""Broker Router — Fase 4: enruta órdenes al broker correcto según AssetClass.

Arquitectura extensible:
  equity_stock / equity_etf / index_option → Tradier
  crypto                                   → ccxt (Binance / Coinbase)
  future / forex                           → esqueleto Fase 5
"""
from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger("atlas.execution.broker_router")

# Asset classes soportadas
from atlas_code_quant.scanner.asset_classifier import AssetClass
from atlas_code_quant.execution.tradier_execution import route_order_to_tradier
from atlas_code_quant.api.schemas import OrderRequest


class BrokerRouter:
    """Enruta una orden al broker adecuado según la clase de activo.

    Uso::
        router = BrokerRouter(mode="paper")
        result = router.route(order_req, asset_class="equity_stock")
    """

    def __init__(self, mode: str = "paper") -> None:
        self.mode = mode.strip().lower()

    def route(self, order_req: OrderRequest, asset_class: str = "equity_stock") -> dict:
        """Despacha la orden al broker correcto. Nunca lanza — retorna dict con status."""
        try:
            ac = AssetClass(asset_class)
        except ValueError:
            ac = AssetClass.EQUITY_STOCK

        # ── Tradier: equities, ETFs, índices ──────────────────────────────
        if ac in (AssetClass.EQUITY_STOCK, AssetClass.EQUITY_ETF, AssetClass.INDEX_OPTION):
            return self._route_tradier(order_req)

        # ── ccxt: crypto ──────────────────────────────────────────────────
        if ac == AssetClass.CRYPTO:
            return self._route_ccxt(order_req)

        # ── Futuros / Forex — fase 5 ──────────────────────────────────────
        if ac in (AssetClass.FUTURE, AssetClass.FOREX):
            return self._route_future_placeholder(order_req, ac)

        # Fallback
        return self._route_tradier(order_req)

    # ── Tradier ────────────────────────────────────────────────────────────────

    def _route_tradier(self, order_req: OrderRequest) -> dict:
        try:
            result = route_order_to_tradier(order_req)
            result["broker"] = "tradier"
            return result
        except Exception as exc:
            logger.error("[BROKER] Tradier error: %s", exc)
            return {"ok": False, "broker": "tradier", "error": str(exc)}

    # ── ccxt / Crypto ──────────────────────────────────────────────────────────

    def _route_ccxt(self, order_req: OrderRequest) -> dict:
        """Ejecuta orden en Binance (paper: sandbox endpoint)."""
        try:
            import ccxt
            exchange_id = os.getenv("ATLAS_CRYPTO_EXCHANGE", "binance")
            api_key     = os.getenv("CCXT_API_KEY", "")
            secret      = os.getenv("CCXT_SECRET", "")

            if not api_key or not secret:
                return self._crypto_simulate(order_req)

            exchange_cls = getattr(ccxt, exchange_id)
            exchange = exchange_cls({
                "apiKey": api_key,
                "secret": secret,
                "sandbox": self.mode == "paper",
                "enableRateLimit": True,
            })

            symbol = order_req.symbol.replace("-", "/")  # BTC-USD → BTC/USD
            side   = order_req.side.lower()
            amount = float(order_req.size)

            if self.mode == "paper":
                return self._crypto_simulate(order_req)

            order = exchange.create_order(
                symbol=symbol,
                type=order_req.order_type or "market",
                side=side,
                amount=amount,
            )
            return {
                "ok":     True,
                "broker": "ccxt_" + exchange_id,
                "order_id": str(order.get("id", "")),
                "status":   order.get("status", "submitted"),
            }

        except ImportError:
            logger.warning("[BROKER] ccxt no instalado — simulando crypto")
            return self._crypto_simulate(order_req)
        except Exception as exc:
            logger.error("[BROKER] ccxt error: %s", exc)
            return {"ok": False, "broker": "ccxt", "error": str(exc)}

    def _crypto_simulate(self, order_req: OrderRequest) -> dict:
        """Simulación local de crypto (paper sin credenciales ccxt)."""
        import time
        return {
            "ok":      True,
            "broker":  "crypto_simulated",
            "order_id": f"CRYPTO-SIM-{int(time.time())}",
            "status":  "simulated",
            "symbol":  order_req.symbol,
            "side":    order_req.side,
            "size":    order_req.size,
        }

    # ── Futuros / Forex (placeholder Fase 5) ──────────────────────────────────

    def _route_future_placeholder(self, order_req: OrderRequest, ac: AssetClass) -> dict:
        logger.warning(
            "[BROKER] %s no implementado aún — simulando localmente", ac.value
        )
        import time
        return {
            "ok":      True,
            "broker":  f"{ac.value}_simulated",
            "order_id": f"{ac.value.upper()}-SIM-{int(time.time())}",
            "status":  "simulated",
            "symbol":  order_req.symbol,
            "note":    "Fase 5 — broker real no conectado",
        }
