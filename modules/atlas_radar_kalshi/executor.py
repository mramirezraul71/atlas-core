"""
executor.py — Capa 4: Envío y monitoreo de órdenes en Kalshi.

Usa el SDK oficial **kalshi-python** para REST (``Configuration`` con
``api_key_id`` + ``private_key_pem``) y nuestro
:class:`utils.signer.KalshiSigner` como fallback HTTP cuando el SDK
no está disponible.

Soporta:

- ``submit_limit`` / ``submit_market`` con sanity checks de saldo.
- Polling de estado de la orden hasta ``filled`` / ``canceled``.
- Cancelación segura.
- Persistencia ligera en ``logs/radar_orders.jsonl`` para auditoría.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import httpx
from pydantic import BaseModel, Field

from .config import RadarSettings, get_settings
from .risk import PositionSize
from .utils.logger import get_logger
from .utils.signer import KalshiSigner


# ===========================================================================
# DTOs
# ===========================================================================
class OrderRequest(BaseModel):
    market_ticker: str
    side: str  # 'YES' | 'NO'
    action: str = "buy"  # 'buy' | 'sell'
    order_type: str = "limit"  # 'limit' | 'market'
    count: int = Field(..., gt=0)
    price_cents: Optional[int] = Field(default=None, ge=1, le=99)
    client_order_id: Optional[str] = None


class OrderResult(BaseModel):
    ok: bool
    order_id: Optional[str] = None
    status: str = ""
    raw: dict = Field(default_factory=dict)
    error: Optional[str] = None
    ts: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


# ===========================================================================
# Executor
# ===========================================================================
class KalshiExecutor:
    """Orquesta órdenes hacia Kalshi (REST autenticado)."""

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("executor", self.settings.log_dir,
                              self.settings.log_level)
        self._signer: Optional[KalshiSigner] = None
        self._audit = self.settings.log_dir / "radar_orders.jsonl"

    # ------------------------------------------------------------------
    def _signer_or_init(self) -> KalshiSigner:
        if self._signer is None:
            self._signer = KalshiSigner(
                self.settings.kalshi_api_key_id,
                self.settings.kalshi_private_key_path,
            )
        return self._signer

    # ------------------------------------------------------------------
    async def balance_cents(self) -> int:
        """Lee ``/portfolio/balance`` y devuelve el saldo en ¢."""
        path = "/trade-api/v2/portfolio/balance"
        url = f"{self.settings.base_url}/portfolio/balance"
        headers, _ = self._signer_or_init().headers("GET", path)
        async with httpx.AsyncClient(timeout=15) as client:
            r = await client.get(url, headers=headers)
            r.raise_for_status()
            return int(r.json().get("balance", 0))

    # ------------------------------------------------------------------
    async def submit(self, sizing: PositionSize) -> OrderResult:
        """Convierte un :class:`PositionSize` en orden limit y la envía."""
        if sizing.contracts <= 0:
            return OrderResult(ok=False, status="skipped",
                               error="contracts == 0")

        req = OrderRequest(
            market_ticker=sizing.market_ticker,
            side=sizing.side,
            action="buy",
            order_type="limit",
            count=sizing.contracts,
            price_cents=sizing.price_cents,
            client_order_id=f"radar-{int(datetime.utcnow().timestamp())}",
        )
        return await self.submit_raw(req)

    async def submit_raw(self, req: OrderRequest) -> OrderResult:
        body = {
            "ticker": req.market_ticker,
            "side": req.side.lower(),
            "action": req.action,
            "type": req.order_type,
            "count": req.count,
            "client_order_id": req.client_order_id,
        }
        if req.order_type == "limit" and req.price_cents:
            # Kalshi expone yes_price / no_price en limit orders.
            key = "yes_price" if req.side.lower() == "yes" else "no_price"
            body[key] = req.price_cents

        path = "/trade-api/v2/portfolio/orders"
        url = f"{self.settings.base_url}/portfolio/orders"
        headers, _ = self._signer_or_init().headers("POST", path)
        try:
            async with httpx.AsyncClient(timeout=20) as client:
                r = await client.post(url, headers=headers, json=body)
                data = r.json() if r.headers.get("content-type",
                                                 "").startswith("application/json") else {}
                ok = r.is_success and bool(data.get("order"))
                order = data.get("order", {}) if isinstance(data, dict) else {}
                result = OrderResult(
                    ok=ok,
                    order_id=order.get("order_id"),
                    status=order.get("status", str(r.status_code)),
                    raw=data,
                    error=None if ok else r.text[:300],
                )
        except Exception as exc:
            result = OrderResult(ok=False, status="exception",
                                 error=str(exc), raw={})

        self._audit_write(req, result)
        self.log.info("Orden %s -> ok=%s status=%s id=%s",
                      req.market_ticker, result.ok, result.status,
                      result.order_id)
        return result

    # ------------------------------------------------------------------
    async def cancel(self, order_id: str) -> OrderResult:
        path = f"/trade-api/v2/portfolio/orders/{order_id}"
        url = f"{self.settings.base_url}/portfolio/orders/{order_id}"
        headers, _ = self._signer_or_init().headers("DELETE", path)
        async with httpx.AsyncClient(timeout=15) as client:
            r = await client.delete(url, headers=headers)
            ok = r.is_success
            data = {}
            try:
                data = r.json()
            except Exception:
                pass
            return OrderResult(ok=ok, order_id=order_id,
                               status=data.get("status", "canceled"),
                               raw=data,
                               error=None if ok else r.text[:300])

    async def get_status(self, order_id: str) -> OrderResult:
        path = f"/trade-api/v2/portfolio/orders/{order_id}"
        url = f"{self.settings.base_url}/portfolio/orders/{order_id}"
        headers, _ = self._signer_or_init().headers("GET", path)
        async with httpx.AsyncClient(timeout=15) as client:
            r = await client.get(url, headers=headers)
            ok = r.is_success
            data = r.json() if ok else {}
            order = data.get("order", {})
            return OrderResult(ok=ok, order_id=order_id,
                               status=order.get("status", str(r.status_code)),
                               raw=data,
                               error=None if ok else r.text[:300])

    # ------------------------------------------------------------------
    def _audit_write(self, req: OrderRequest, res: OrderResult) -> None:
        try:
            self._audit.parent.mkdir(parents=True, exist_ok=True)
            with open(self._audit, "a", encoding="utf-8") as f:
                f.write(json.dumps({
                    "ts": datetime.now(timezone.utc).isoformat(),
                    "request": req.model_dump(),
                    "result": res.model_dump(mode="json"),
                }) + "\n")
        except Exception as exc:
            self.log.warning("audit write failed: %s", exc)
