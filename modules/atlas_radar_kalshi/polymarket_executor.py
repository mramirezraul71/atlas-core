from __future__ import annotations

import asyncio
import json
import time
from typing import Any, Optional, Tuple

import httpx
from pydantic import BaseModel

from .config import RadarSettings, get_settings
from .executor_v2 import FillReport, OrderRequestV2
from .utils.logger import get_logger


class PolymarketExecutorConfig(BaseModel):
    enable_live: bool = False


def _parse_poly_ticker(market_ticker: str) -> str:
    t = (market_ticker or "").strip().upper()
    if not t.startswith("POLY:"):
        raise ValueError("invalid_polymarket_ticker")
    mid = t.split(":", 1)[1].strip()
    if not mid:
        raise ValueError("invalid_polymarket_ticker")
    return mid


def _clob_yes_no_tokens(market: dict[str, Any]) -> Tuple[str, str]:
    raw = market.get("clobTokenIds")
    if not raw:
        raise ValueError("no_clob_token_ids")
    if isinstance(raw, str):
        tokens = json.loads(raw)
    elif isinstance(raw, list):
        tokens = raw
    else:
        raise ValueError("clob_token_ids_invalid")
    if not isinstance(tokens, list) or len(tokens) < 2:
        raise ValueError("clob_token_ids_invalid")
    return str(tokens[0]), str(tokens[1])


def _round_to_tick(price: float, tick: float) -> float:
    if tick <= 0:
        return float(price)
    n = round(float(price) / tick)
    return round(n * tick, 10)


class PolymarketExecutor:
    """Executor Polymarket: paper emulado; live vía CLOB firmado (py-clob-client)."""

    def __init__(
        self,
        settings: Optional[RadarSettings] = None,
        cfg: Optional[PolymarketExecutorConfig] = None,
    ) -> None:
        self.settings = settings or get_settings()
        self.cfg = cfg or PolymarketExecutorConfig(
            enable_live=self.settings.execution_mode == "live"
        )
        self.log = get_logger("polymarket_executor", self.settings.log_dir, self.settings.log_level)

    async def _fetch_gamma_market(self, market_id: str) -> dict[str, Any]:
        base = self.settings.polymarket_gamma_url.rstrip("/")
        url = f"{base}/markets/{market_id}"
        async with httpx.AsyncClient(timeout=25) as client:
            r = await client.get(url)
            r.raise_for_status()
        data = r.json()
        if not isinstance(data, dict):
            raise ValueError("gamma_market_invalid")
        return data

    def _build_clob_client(self):  # type: ignore[no-untyped-def]
        try:
            from py_clob_client.client import ClobClient
        except ImportError as exc:
            raise RuntimeError("py_clob_client_not_installed") from exc
        pk = (self.settings.polymarket_private_key or "").strip()
        if not pk:
            raise RuntimeError("polymarket_private_key_missing")
        host = self.settings.polymarket_clob_url.rstrip("/")
        kwargs: dict[str, Any] = {
            "host": host,
            "chain_id": int(self.settings.polymarket_chain_id),
            "key": pk,
        }
        st = self.settings.polymarket_signature_type
        if st is not None:
            kwargs["signature_type"] = int(st)
        fund = (self.settings.polymarket_funder or "").strip()
        if fund:
            kwargs["funder"] = fund
        return ClobClient(**kwargs)

    def _place_limit_sync(
        self,
        *,
        token_id: str,
        price: float,
        size: float,
    ) -> Any:
        from py_clob_client.clob_types import OrderArgs, OrderType
        from py_clob_client.order_builder.constants import BUY

        client = self._build_clob_client()
        client.set_api_creds(client.create_or_derive_api_creds())
        order = OrderArgs(
            token_id=token_id,
            price=price,
            size=size,
            side=BUY,
        )
        signed = client.create_order(order)
        return client.post_order(signed, OrderType.GTC)

    async def submit(self, req: OrderRequestV2) -> FillReport:
        t0 = time.perf_counter()
        if not self.cfg.enable_live:
            lat = int((time.perf_counter() - t0) * 1000)
            return FillReport(
                ok=True,
                order_id=f"poly-paper-{req.client_order_id}",
                filled_contracts=req.contracts,
                avg_fill_price=float(req.price_cents),
                status="filled",
                latency_ms=lat,
                slippage_cents=0.0,
            )
        if not (self.settings.polymarket_private_key or "").strip():
            return FillReport(
                ok=False,
                order_id=req.client_order_id,
                filled_contracts=0,
                avg_fill_price=0.0,
                status="rejected",
                latency_ms=int((time.perf_counter() - t0) * 1000),
                error="polymarket_private_key_missing",
            )
        try:
            market_id = _parse_poly_ticker(req.market_ticker)
            m = await self._fetch_gamma_market(market_id)
            yes_t, no_t = _clob_yes_no_tokens(m)
            side_u = (req.side or "YES").strip().upper()
            token_id = yes_t if side_u in ("YES", "Y", "1") else no_t

            tick = float(m.get("orderPriceMinTickSize") or 0.01)
            p = _round_to_tick((req.price_cents or 0) / 100.0, tick)
            if p <= 0 or p >= 1:
                return FillReport(
                    ok=False,
                    order_id=req.client_order_id,
                    filled_contracts=0,
                    avg_fill_price=0.0,
                    status="rejected",
                    latency_ms=int((time.perf_counter() - t0) * 1000),
                    error="invalid_price",
                    raw={"price": p, "tick": tick},
                )
            min_sz = float(m.get("orderMinSize") or 0)
            sz = float(req.contracts)
            if min_sz and sz < min_sz:
                return FillReport(
                    ok=False,
                    order_id=req.client_order_id,
                    filled_contracts=0,
                    avg_fill_price=0.0,
                    status="rejected",
                    latency_ms=int((time.perf_counter() - t0) * 1000),
                    error="below_order_min_size",
                    raw={"min_size": min_sz, "size": sz},
                )
        except Exception as exc:
            return FillReport(
                ok=False,
                order_id=req.client_order_id,
                filled_contracts=0,
                avg_fill_price=0.0,
                status="error",
                latency_ms=int((time.perf_counter() - t0) * 1000),
                error=str(exc)[:200],
            )

        loop = asyncio.get_event_loop()
        try:
            resp = await loop.run_in_executor(
                None,
                lambda: self._place_limit_sync(
                    token_id=token_id,
                    price=p,
                    size=sz,
                ),
            )
        except RuntimeError as exc:
            err = str(exc)
            if "py_clob_client" in err or "not_installed" in err:
                err = "py_clob_client_not_installed"
            return FillReport(
                ok=False,
                order_id=req.client_order_id,
                filled_contracts=0,
                avg_fill_price=0.0,
                status="error",
                latency_ms=int((time.perf_counter() - t0) * 1000),
                error=err,
            )
        except Exception as exc:
            return FillReport(
                ok=False,
                order_id=req.client_order_id,
                filled_contracts=0,
                avg_fill_price=0.0,
                status="error",
                latency_ms=int((time.perf_counter() - t0) * 1000),
                error=str(exc)[:200],
            )

        lat = int((time.perf_counter() - t0) * 1000)
        ok = True
        oid: Optional[str] = None
        if isinstance(resp, dict):
            if resp.get("error") or resp.get("errorMsg") or resp.get("errMsg"):
                ok = False
            if resp.get("success") is False:
                ok = False
            oid = str(resp.get("orderID") or resp.get("orderId") or resp.get("id") or "") or None
        price_out = p * 100.0
        if isinstance(resp, dict) and resp.get("avgPrice") is not None:
            try:
                price_out = float(resp["avgPrice"]) * 100.0
            except Exception:
                pass
        return FillReport(
            ok=ok,
            order_id=oid or f"poly-live-{req.client_order_id}",
            filled_contracts=int(req.contracts) if ok else 0,
            avg_fill_price=price_out,
            status="submitted" if ok else "rejected",
            latency_ms=lat,
            slippage_cents=0.0,
            raw=resp if isinstance(resp, dict) else {"raw": str(resp)[:2000]},
        )
