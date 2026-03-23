"""Alpaca Markets — adaptador paper trading.

Actúa como fallback cuando Tradier sandbox (sandbox.tradier.com) no está
disponible (504, 503, ConnectionError).

API: https://paper-api.alpaca.markets/v2
Autenticación: headers APCA-API-KEY-ID + APCA-API-SECRET-KEY
Documentación: https://docs.alpaca.markets/reference/postorder

Credenciales gratuitas en: https://app.alpaca.markets → Paper Trading
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger("atlas.execution.alpaca_paper")

_ALPACA_PAPER_BASE = "https://paper-api.alpaca.markets/v2"
_TIMEOUT_SEC = int(os.getenv("ALPACA_TIMEOUT_SEC", "10"))

# Errores de Tradier que disparan el fallback a Alpaca
_TRADIER_FALLBACK_ERRORS = (
    "504",
    "503",
    "502",
    "gateway",
    "timeout",
    "connectionerror",
    "connection error",
    "timed out",
    "sandbox.tradier.com",
)


def _tradier_error_is_fallback(exc: Exception) -> bool:
    """True si el error de Tradier justifica intentar Alpaca."""
    msg = str(exc).lower()
    return any(kw in msg for kw in _TRADIER_FALLBACK_ERRORS)


@dataclass
class AlpacaOrderResult:
    order_id:   str  = ""
    status:     str  = "pending"     # accepted | filled | rejected | error
    symbol:     str  = ""
    side:       str  = ""
    qty:        int  = 0
    fill_price: float = 0.0
    raw:        dict = field(default_factory=dict)
    error:      str  = ""
    broker:     str  = "alpaca_paper"


class AlpacaPaperBroker:
    """Broker paper de Alpaca.  Interfaz mínima compatible con SignalExecutor.

    Uso::

        broker = AlpacaPaperBroker(api_key="PKXXX", secret_key="YYY")
        result = broker.place_order("AAPL", side="buy", qty=10)
    """

    def __init__(
        self,
        api_key: str = "",
        secret_key: str = "",
        base_url: str = _ALPACA_PAPER_BASE,
    ) -> None:
        self.api_key    = api_key    or os.getenv("ALPACA_API_KEY", "").strip()
        self.secret_key = secret_key or os.getenv("ALPACA_SECRET_KEY", "").strip()
        self.base_url   = base_url.rstrip("/")
        self._available: bool | None = None   # cache del health check

    # ── API pública ───────────────────────────────────────────────────────────

    def is_configured(self) -> bool:
        """True si las credenciales están cargadas."""
        return bool(self.api_key and self.secret_key)

    def health_check(self, force: bool = False) -> bool:
        """Verifica conectividad con Alpaca paper. Cachea resultado."""
        if self._available is not None and not force:
            return self._available
        try:
            import requests
            r = requests.get(
                f"{self.base_url}/account",
                headers=self._headers(),
                timeout=_TIMEOUT_SEC,
            )
            self._available = r.status_code == 200
        except Exception as exc:
            logger.debug("Alpaca health_check falló: %s", exc)
            self._available = False
        return self._available  # type: ignore[return-value]

    def place_order(
        self,
        symbol: str,
        side: str,          # "buy" | "sell"
        qty: int,
        order_type: str = "market",
        time_in_force: str = "day",
        limit_price: float | None = None,
    ) -> AlpacaOrderResult:
        """Envía orden al paper account de Alpaca."""
        result = AlpacaOrderResult(symbol=symbol, side=side, qty=qty)

        if not self.is_configured():
            result.status = "error"
            result.error  = "Alpaca credentials no configuradas (ALPACA_API_KEY / ALPACA_SECRET_KEY)"
            logger.warning("[ALPACA] %s", result.error)
            return result

        payload: dict[str, Any] = {
            "symbol":        symbol.upper(),
            "qty":           str(qty),
            "side":          side.lower(),
            "type":          order_type,
            "time_in_force": time_in_force,
        }
        if order_type == "limit" and limit_price:
            payload["limit_price"] = str(round(limit_price, 2))

        try:
            import requests
            r = requests.post(
                f"{self.base_url}/orders",
                json=payload,
                headers=self._headers(),
                timeout=_TIMEOUT_SEC,
            )
            data = r.json() if r.content else {}
            result.raw = data

            if r.status_code in (200, 201):
                result.order_id   = data.get("id", "")
                result.status     = data.get("status", "accepted")
                result.fill_price = float(data.get("filled_avg_price") or 0.0)
                logger.info(
                    "[ALPACA] ORDEN %s %s %s qty=%d id=%s status=%s",
                    side.upper(), symbol, order_type, qty,
                    result.order_id, result.status,
                )
            else:
                result.status = "rejected"
                result.error  = data.get("message") or f"HTTP {r.status_code}"
                logger.warning("[ALPACA] Orden rechazada %s: %s", symbol, result.error)

        except Exception as exc:
            result.status = "error"
            result.error  = str(exc)
            logger.error("[ALPACA] Error enviando orden %s: %s", symbol, exc)

        return result

    def get_account(self) -> dict:
        """Devuelve datos de la cuenta paper de Alpaca."""
        if not self.is_configured():
            return {}
        try:
            import requests
            r = requests.get(
                f"{self.base_url}/account",
                headers=self._headers(),
                timeout=_TIMEOUT_SEC,
            )
            return r.json() if r.status_code == 200 else {}
        except Exception:
            return {}

    def get_positions(self) -> list[dict]:
        """Lista posiciones abiertas en la cuenta paper."""
        if not self.is_configured():
            return []
        try:
            import requests
            r = requests.get(
                f"{self.base_url}/positions",
                headers=self._headers(),
                timeout=_TIMEOUT_SEC,
            )
            return r.json() if r.status_code == 200 else []
        except Exception:
            return []

    def cancel_order(self, order_id: str) -> bool:
        """Cancela una orden por ID. Retorna True si se canceló."""
        if not self.is_configured() or not order_id:
            return False
        try:
            import requests
            r = requests.delete(
                f"{self.base_url}/orders/{order_id}",
                headers=self._headers(),
                timeout=_TIMEOUT_SEC,
            )
            return r.status_code in (200, 204)
        except Exception:
            return False

    # ── Internos ──────────────────────────────────────────────────────────────

    def _headers(self) -> dict[str, str]:
        return {
            "APCA-API-KEY-ID":     self.api_key,
            "APCA-API-SECRET-KEY": self.secret_key,
            "Content-Type":        "application/json",
            "Accept":              "application/json",
        }

    def __repr__(self) -> str:  # pragma: no cover
        configured = "configured" if self.is_configured() else "no_credentials"
        return f"AlpacaPaperBroker({self.base_url}, {configured})"


# ── Singleton lazy ─────────────────────────────────────────────────────────────

_broker_instance: AlpacaPaperBroker | None = None


def get_alpaca_paper_broker() -> AlpacaPaperBroker:
    """Retorna instancia singleton del broker Alpaca paper."""
    global _broker_instance
    if _broker_instance is None:
        from config.settings import settings
        _broker_instance = AlpacaPaperBroker(
            api_key    = getattr(settings, "alpaca_api_key",    ""),
            secret_key = getattr(settings, "alpaca_secret_key", ""),
        )
    return _broker_instance
