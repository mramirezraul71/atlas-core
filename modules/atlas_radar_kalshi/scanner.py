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
import os
import time
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
        self._queue: asyncio.Queue[MarketEvent] = asyncio.Queue(
            maxsize=int(self.settings.event_queue_maxsize)
        )
        # Histórico ligero por ticker para Markov / Monte Carlo
        self._history: Dict[str, pd.DataFrame] = {}
        self._active_ws = None
        self._last_book_emit_ts: Dict[str, float] = {}
        self._last_ticker_emit_ts: Dict[str, float] = {}
        self._dropped_events: int = 0
        self._new_market_events_emitted: int = 0
        self._discover_lock = asyncio.Lock()
        self.uses_public_rest_feed = self._public_feed_allowed()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _credentials_ready(self) -> bool:
        kid = (self.settings.kalshi_api_key_id or "").strip()
        if not kid:
            return False
        return self.settings.kalshi_private_key_path.expanduser().exists()

    def _public_feed_allowed(self) -> bool:
        """REST público Kalshi (sin firma) cuando no hay key+PEM."""
        if self._credentials_ready():
            return False
        return os.getenv("RADAR_DISABLE_PUBLIC_MARKET_DATA", "0") != "1"

    def refresh_feed_mode(self) -> None:
        """Tras cambiar settings en caliente (dashboard)."""
        self.uses_public_rest_feed = self._public_feed_allowed()

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

    def _enqueue_event(self, event: MarketEvent) -> bool:
        """Encola evento sin bloquear el loop WS; si hay presión, recorta cola."""
        try:
            self._queue.put_nowait(event)
            return True
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()  # descarta el más viejo
            except asyncio.QueueEmpty:
                pass
            try:
                self._queue.put_nowait(event)
                self._dropped_events += 1
                if self._dropped_events == 1 or self._dropped_events % 200 == 0:
                    self.log.warning(
                        "Queue pressure: eventos recortados=%d (max=%d)",
                        self._dropped_events,
                        self._queue.maxsize,
                    )
                return True
            except asyncio.QueueFull:
                self._dropped_events += 1
                if self._dropped_events == 1 or self._dropped_events % 200 == 0:
                    self.log.warning(
                        "Queue full: evento descartado (recortados=%d, max=%d)",
                        self._dropped_events,
                        self._queue.maxsize,
                    )
                return False

    # ------------------------------------------------------------------
    # REST: descubrimiento de mercados
    # ------------------------------------------------------------------
    async def discover_markets(self) -> list[str]:
        """Sondea ``GET /markets`` y devuelve tickers nuevos."""
        async with self._discover_lock:
            url = f"{self.settings.base_url}/markets"
            params = {
                "limit": int(self.settings.market_discovery_limit),
                "status": self.settings.market_status_filter,
            }
            if self._credentials_ready():
                signer = self._signer_or_init()
                headers, _ = signer.headers("GET", "/trade-api/v2/markets")
            else:
                headers = {"Accept": "application/json"}

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
                    if self._new_market_events_emitted < int(self.settings.new_market_events_max):
                        self._enqueue_event(MarketEvent(
                            kind="new_market", market_ticker=t, payload=m
                        ))
                        self._new_market_events_emitted += 1
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

    async def _public_orderbook_poll_loop(self) -> None:
        """
        Sin credenciales Kalshi, el WS firmado no está disponible; la API
        pública permite ``GET /markets/{ticker}/orderbook`` sin headers de firma
        (Kalshi docs: market data quick start).
        """
        max_ob = max(1, min(25, int(self.settings.ws_max_tickers)))
        interval = max(2.5, min(12.0, float(self.settings.poll_interval_seconds) / 3.0))
        while True:
            try:
                tickers = list(self._tickers)[:max_ob]
                if not tickers:
                    await asyncio.sleep(1.0)
                    continue
                await self.pulse_public_orderbooks(limit=max_ob)
            except Exception as exc:
                self.log.warning("public orderbook poll: %s", exc)
            await asyncio.sleep(interval)

    async def pulse_public_orderbooks(self, limit: int = 25) -> int:
        """
        Fuerza un barrido puntual de orderbooks públicos y emite eventos.

        Se usa tanto en el loop normal como en recuperación del watchdog.
        """
        if not self.uses_public_rest_feed:
            return 0
        tickers = list(self._tickers)[: max(1, int(limit))]
        if not tickers:
            return 0
        emitted = 0
        base = str(self.settings.base_url).rstrip("/")
        async with httpx.AsyncClient(timeout=25) as client:
            for t in tickers:
                try:
                    r = await client.get(
                        f"{base}/markets/{t}/orderbook",
                        headers={"Accept": "application/json"},
                    )
                    if r.status_code != 200:
                        continue
                    data = r.json()
                    ob = data.get("orderbook_fp") or {}
                    yes_raw = ob.get("yes_dollars") or []
                    no_raw = ob.get("no_dollars") or []
                    if not yes_raw and not no_raw:
                        continue
                    yes_levels: list[list[int]] = []
                    for row in yes_raw:
                        if len(row) >= 2:
                            price_d, qv = row[0], row[1]
                            pc = int(round(float(price_d) * 100))
                            pc = max(1, min(99, pc))
                            yes_levels.append([pc, max(1, int(float(qv)))])
                    no_levels: list[list[int]] = []
                    for row in no_raw:
                        if len(row) >= 2:
                            price_d, qv = row[0], row[1]
                            pc = int(round(float(price_d) * 100))
                            pc = max(1, min(99, pc))
                            no_levels.append([pc, max(1, int(float(qv)))])
                    book = self._books.setdefault(t, _LocalOrderBook(t))
                    book.apply_snapshot({"yes": yes_levels, "no": no_levels})
                    await self._emit_book(book)
                    emitted += 1
                except Exception as exc:
                    self.log.debug("orderbook %s: %s", t, exc)
        return emitted

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
            tickers = list(self._tickers)
            if not tickers:
                await self.discover_markets()
                tickers = list(self._tickers)
            max_tickers = int(self.settings.ws_max_tickers)
            selected = tickers[:max_tickers]
            if not selected:
                self.log.warning(
                    "Sin tickers para suscribir; esperando siguiente ciclo de descubrimiento."
                )
                return
            sub = {
                "id": 1,
                "cmd": "subscribe",
                "params": {
                    "channels": ["orderbook_delta", "ticker"],
                    "market_tickers": selected,
                },
            }
            await ws.send(json.dumps(sub))
            self.log.info("Suscrito a %d tickers", len(selected))

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
            now = time.time()
            min_gap = max(0.0, float(self.settings.ticker_emit_min_ms) / 1000.0)
            last = self._last_ticker_emit_ts.get(ticker, 0.0)
            if now - last >= min_gap:
                self._last_ticker_emit_ts[ticker] = now
                self._enqueue_event(MarketEvent(
                    kind="ticker", market_ticker=ticker, payload=body
                ))
                self._record_history(ticker, body)

        elif msg_type == "trade" and ticker:
            self._enqueue_event(MarketEvent(
                kind="trade", market_ticker=ticker, payload=body
            ))

        elif msg_type == "subscribed":
            self.log.debug("Suscripción confirmada: %s", body)
        elif msg_type == "error":
            self.log.warning("WS ERROR: %s", msg)

    async def _emit_book(self, book: _LocalOrderBook) -> None:
        now = time.time()
        min_gap = max(0.0, float(self.settings.book_emit_min_ms) / 1000.0)
        last = self._last_book_emit_ts.get(book.market_ticker, 0.0)
        if now - last < min_gap:
            return
        self._last_book_emit_ts[book.market_ticker] = now
        snap = book.to_snapshot()
        self._enqueue_event(MarketEvent(
            kind="orderbook",
            market_ticker=book.market_ticker,
            payload=snap.model_dump(mode="json"),
        ))

    @staticmethod
    def _as_cents(value: object) -> Optional[int]:
        if value is None:
            return None
        try:
            v = float(value)
        except Exception:
            return None
        cents = int(round(v * 100)) if abs(v) <= 1.0 else int(round(v))
        return max(0, min(99, cents))

    def _record_history(self, ticker: str, body: dict) -> None:
        df = self._history.get(ticker)
        bid = self._as_cents(body.get("yes_bid"))
        if bid is None:
            bid = self._as_cents(body.get("yes_bid_dollars"))
        ask = self._as_cents(body.get("yes_ask"))
        if ask is None:
            ask = self._as_cents(body.get("yes_ask_dollars"))
        if bid is None and ask is None:
            return
        if bid is None:
            bid = max(1, int(ask or 50) - 1)
        if ask is None:
            ask = min(99, int(bid) + 1)
        row = {
            "ts": datetime.now(timezone.utc),
            "yes_mid": (bid + ask) / 200.0,
            "yes_bid": bid,
            "yes_ask": ask,
        }
        new_df = pd.DataFrame([row])
        self._history[ticker] = (
            new_df if df is None else pd.concat([df, new_df], ignore_index=True)
        ).tail(2000)

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------
    async def stream(self) -> AsyncIterator[MarketEvent]:
        """Inicia REST poller + (WS firmado o feed público de libros)."""
        self.refresh_feed_mode()
        creds = self._credentials_ready()
        tasks: list[asyncio.Task[None]] = []
        if creds:
            tasks.append(
                asyncio.create_task(self._ws_loop(), name="radar-ws")
            )
        else:
            self.log.info(
                "Kalshi sin key+PEM: mercados y libros vía REST público "
                "(WS firmado desactivado). Opcional: RADAR_DISABLE_PUBLIC_MARKET_DATA=1 "
                "para forzar error si faltan credenciales."
            )
        tasks.append(
            asyncio.create_task(self._rest_polling_loop(), name="radar-rest")
        )
        if self.uses_public_rest_feed:
            tasks.append(
                asyncio.create_task(
                    self._public_orderbook_poll_loop(), name="radar-pub-ob"
                )
            )
        try:
            while True:
                ev = await self._queue.get()
                yield ev
        finally:
            for t in tasks:
                t.cancel()

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
