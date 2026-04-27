"""
scanner.py — Capa 1: Escaneo de mercados Kalshi.

Funciones principales
---------------------
1. **WebSocket listener** (``wss://.../trade-api/ws/v2``) para recibir
   en tiempo real:
       * ``orderbook_snapshot`` y ``orderbook_delta``
       * ``ticker`` (público)
       * ``trade``
   Mantiene un **OrderBook local** por mercado.

2. **REST poller** (``GET /markets``) que detecta nuevos mercados y los
   incorpora a la lista de suscripciones.

3. **Bus interno** (``asyncio.Queue``) que publica eventos
   normalizados :class:`MarketEvent` para que el ``brain`` decida.

Notas de diseño
---------------
- No usamos directamente el cliente de alto nivel ``kalshi-python``
  para el WebSocket porque a la fecha el endpoint v2 firma el path y
  expone canales que se manejan mejor con la lib ``websockets``.
  Mantenemos ``kalshi-python`` como cliente REST oficial (ver
  ``executor.py``).
"""
from __future__ import annotations

import asyncio
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import AsyncIterator, Awaitable, Callable, Dict, Optional

import httpx
import pandas as pd
import websockets
from pydantic import BaseModel, Field

from .config import RadarSettings, get_settings
from .utils.logger import get_logger
from .utils.signer import KalshiSigner


# ===========================================================================
# Modelos tipados
# ===========================================================================
class OrderBookSnapshot(BaseModel):
    """Snapshot inmutable de un orderbook agregado por niveles (ticks)."""
    market_ticker: str
    yes_bids: list[tuple[int, int]] = Field(
        default_factory=list,
        description="Lista [(precio_¢, qty)] para YES.",
    )
    yes_asks: list[tuple[int, int]] = Field(default_factory=list)
    no_bids: list[tuple[int, int]] = Field(default_factory=list)
    no_asks: list[tuple[int, int]] = Field(default_factory=list)
    ts: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))

    @property
    def yes_mid(self) -> Optional[float]:
        """Precio medio implícito YES (en probabilidad 0-1)."""
        if not self.yes_bids or not self.yes_asks:
            return None
        best_bid = max(p for p, _ in self.yes_bids)
        best_ask = min(p for p, _ in self.yes_asks)
        return ((best_bid + best_ask) / 2) / 100.0


class MarketEvent(BaseModel):
    """Evento normalizado emitido al bus interno."""
    kind: str  # 'orderbook' | 'ticker' | 'trade' | 'lifecycle' | 'new_market'
    market_ticker: str
    payload: dict
    ts: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


# ===========================================================================
# OrderBook mutable (estado interno)
# ===========================================================================
@dataclass
class _LocalOrderBook:
    """OrderBook local actualizable a partir de deltas."""
    market_ticker: str
    yes: Dict[int, int] = field(default_factory=dict)  # bids YES (price->qty)
    no: Dict[int, int] = field(default_factory=dict)   # bids NO

    def apply_snapshot(self, msg: dict) -> None:
        self.yes.clear()
        self.no.clear()
        for price, qty in msg.get("yes", []):
            self.yes[int(price)] = int(qty)
        for price, qty in msg.get("no", []):
            self.no[int(price)] = int(qty)

    def apply_delta(self, msg: dict) -> None:
        side = msg.get("side", "yes")
        price = int(msg.get("price", 0))
        delta = int(msg.get("delta", 0))
        book = self.yes if side == "yes" else self.no
        book[price] = book.get(price, 0) + delta
        if book[price] <= 0:
            book.pop(price, None)

    def to_snapshot(self) -> OrderBookSnapshot:
        # Kalshi YES bids son la oferta de compra YES (precio en ¢ 1-99)
        # YES asks ≈ 100 - NO bids (la "venta" de YES equivale a comprar NO).
        yes_bids = sorted(self.yes.items(), key=lambda x: -x[0])
        no_bids = sorted(self.no.items(), key=lambda x: -x[0])
        yes_asks = [(100 - p, q) for p, q in no_bids]
        no_asks = [(100 - p, q) for p, q in yes_bids]
        return OrderBookSnapshot(
            market_ticker=self.market_ticker,
            yes_bids=yes_bids,
            yes_asks=yes_asks,
            no_bids=no_bids,
            no_asks=no_asks,
        )


# ===========================================================================
# Scanner
# ===========================================================================
class KalshiScanner:
    """
    Escáner híbrido REST + WebSocket.

    Uso típico::

        async def main():
            scanner = KalshiScanner()
            async for ev in scanner.stream():
                await brain.on_event(ev)
    """

    WS_PATH = "/trade-api/ws/v2"

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("scanner", self.settings.log_dir,
                              self.settings.log_level)
        self._signer: Optional[KalshiSigner] = None
        self._books: Dict[str, _LocalOrderBook] = {}
        self._tickers: set[str] = set()
        self._queue: asyncio.Queue[MarketEvent] = asyncio.Queue(maxsize=10_000)
        # Histórico ligero por ticker para Markov / Monte Carlo
        self._history: Dict[str, pd.DataFrame] = {}
        self._active_ws = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _signer_or_init(self) -> KalshiSigner:
        if self._signer is None:
            self._signer = KalshiSigner(
                self.settings.kalshi_api_key_id,
                self.settings.kalshi_private_key_path,
            )
        return self._signer

    def history(self, ticker: str) -> pd.DataFrame:
        """Devuelve la serie temporal observada para un ticker."""
        return self._history.get(ticker, pd.DataFrame(
            columns=["ts", "yes_mid", "yes_bid", "yes_ask"]
        ))

    # ------------------------------------------------------------------
    # REST: descubrimiento de mercados
    # ------------------------------------------------------------------
    async def discover_markets(self) -> list[str]:
        """Sondea ``GET /markets`` y devuelve tickers nuevos."""
        url = f"{self.settings.base_url}/markets"
        params = {"limit": 200, "status": self.settings.market_status_filter}
        signer = self._signer_or_init()
        headers, _ = signer.headers("GET", "/trade-api/v2/markets")

        async with httpx.AsyncClient(timeout=20) as client:
            r = await client.get(url, headers=headers, params=params)
            r.raise_for_status()
            data = r.json()

        new = []
        for m in data.get("markets", []):
            t = m.get("ticker")
            if t and t not in self._tickers:
                self._tickers.add(t)
                new.append(t)
                await self._queue.put(MarketEvent(
                    kind="new_market", market_ticker=t, payload=m
                ))
        if new:
            self.log.info("Nuevos mercados detectados: %s", new[:5])
        return new

    async def _rest_polling_loop(self) -> None:
        """Loop de descubrimiento periódico."""
        while True:
            try:
                await self.discover_markets()
            except Exception as exc:
                self.log.warning("REST polling falló: %s", exc)
            await asyncio.sleep(self.settings.poll_interval_seconds)

    # ------------------------------------------------------------------
    # WebSocket: orderbook + ticker
    # ------------------------------------------------------------------
    async def _ws_loop(self) -> None:
        """Loop con reconexión exponencial."""
        backoff = 1.0
        while True:
            try:
                await self._ws_session()
                backoff = 1.0
            except Exception as exc:
                self.log.error("WS error: %s — reintento en %.1fs", exc, backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 60.0)

    async def _ws_session(self) -> None:
        signer = self._signer_or_init()
        headers, _ = signer.headers("GET", self.WS_PATH)

        self.log.info("Conectando WebSocket a %s", self.settings.ws_url)
        async with websockets.connect(
            self.settings.ws_url, additional_headers=headers,
            ping_interval=20, ping_timeout=20, max_size=2**22,
        ) as ws:
            self._active_ws = ws
            # Suscripción inicial
            tickers = list(self._tickers) or await self.discover_markets()
            sub = {
                "id": 1,
                "cmd": "subscribe",
                "params": {
                    "channels": ["orderbook_delta", "ticker"],
                    "market_tickers": tickers[:200],
                },
            }
            await ws.send(json.dumps(sub))
            self.log.info("Suscrito a %d tickers", len(tickers[:200]))

            try:
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    await self._handle_ws_message(msg)
            finally:
                self._active_ws = None

    async def _handle_ws_message(self, msg: dict) -> None:
        msg_type = msg.get("type")
        body = msg.get("msg", {}) or {}
        ticker = body.get("market_ticker") or body.get("ticker")

        if msg_type == "orderbook_snapshot" and ticker:
            book = self._books.setdefault(ticker, _LocalOrderBook(ticker))
            book.apply_snapshot(body)
            await self._emit_book(book)

        elif msg_type == "orderbook_delta" and ticker:
            book = self._books.setdefault(ticker, _LocalOrderBook(ticker))
            book.apply_delta(body)
            await self._emit_book(book)

        elif msg_type == "ticker" and ticker:
            await self._queue.put(MarketEvent(
                kind="ticker", market_ticker=ticker, payload=body
            ))
            self._record_history(ticker, body)

        elif msg_type == "trade" and ticker:
            await self._queue.put(MarketEvent(
                kind="trade", market_ticker=ticker, payload=body
            ))

        elif msg_type == "subscribed":
            self.log.debug("Suscripción confirmada: %s", body)
        elif msg_type == "error":
            self.log.warning("WS ERROR: %s", msg)

    async def _emit_book(self, book: _LocalOrderBook) -> None:
        snap = book.to_snapshot()
        await self._queue.put(MarketEvent(
            kind="orderbook",
            market_ticker=book.market_ticker,
            payload=snap.model_dump(mode="json"),
        ))

    def _record_history(self, ticker: str, body: dict) -> None:
        df = self._history.get(ticker)
        row = {
            "ts": datetime.now(timezone.utc),
            "yes_mid": (body.get("yes_bid", 0) + body.get("yes_ask", 0)) / 200.0,
            "yes_bid": body.get("yes_bid"),
            "yes_ask": body.get("yes_ask"),
        }
        new_df = pd.DataFrame([row])
        self._history[ticker] = (
            new_df if df is None else pd.concat([df, new_df], ignore_index=True)
        ).tail(2000)

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------
    async def stream(self) -> AsyncIterator[MarketEvent]:
        """Inicia REST poller + WS y emite eventos al consumidor."""
        ws_task = asyncio.create_task(self._ws_loop(), name="radar-ws")
        rest_task = asyncio.create_task(self._rest_polling_loop(),
                                        name="radar-rest")
        try:
            while True:
                ev = await self._queue.get()
                yield ev
        finally:
            ws_task.cancel()
            rest_task.cancel()

    async def run_forever(
        self, callback: Callable[[MarketEvent], Awaitable[None]]
    ) -> None:
        """Variante con callback (estilo handler) en vez de async-for."""
        async for ev in self.stream():
            try:
                await callback(ev)
            except Exception as exc:
                self.log.exception("Callback falló: %s", exc)

    async def reconnect(self) -> None:
        """Fuerza reconexión del WS activo (útil tras cambiar entorno/key)."""
        ws = self._active_ws
        if ws is None:
            return
        try:
            await ws.close(code=1012, reason="radar-reconfigure")
        except Exception as exc:
            self.log.debug("No se pudo cerrar WS activo para reconectar: %s", exc)
