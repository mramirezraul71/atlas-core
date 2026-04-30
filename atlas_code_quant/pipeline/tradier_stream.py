"""Módulo 2A — Tradier WebSocket Streaming + Options Chain REST.

Tradier Brokerage Streaming API:
  - Autenticación: Bearer token (live o sandbox)
  - Endpoint stream: wss://stream.tradier.com/v1/markets/events
  - Crea sesión primero: POST /v1/markets/events/session
  - Eventos: quote | trade | summary | timesale | tradex

Uso::

    client = TradierStreamClient(token="...", sandbox=True)
    client.subscribe(["AAPL", "SPY"])
    client.on_quote(lambda q: print(q))
    client.on_trade(lambda t: print(t))
    client.connect()   # bloquea en thread separado
    # ...
    client.disconnect()
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import requests

logger = logging.getLogger("atlas.pipeline.stream")

try:
    import websocket
    _WS_OK = True
except ImportError:
    _WS_OK = False
    logger.warning("websocket-client no instalado. pip install websocket-client")


# ── Estructuras de datos ──────────────────────────────────────────────────────

@dataclass
class StreamQuote:
    symbol: str
    bid: float
    ask: float
    bid_size: int
    ask_size: int
    timestamp: float
    iv: float = 0.0         # IV si es opción
    delta: float = 0.0
    open_interest: int = 0


@dataclass
class StreamTrade:
    symbol: str
    price: float
    size: int
    timestamp: float
    side: str = ""          # "buy" | "sell" (estimado por Lee-Ready)
    exchange: str = ""


@dataclass
class OptionsChain:
    symbol: str
    expiration: str
    calls: list[dict] = field(default_factory=list)
    puts: list[dict] = field(default_factory=list)
    atm_iv: float = 0.0
    iv_rank: float = 0.0
    iv_hv_ratio: float = 0.0
    put_call_skew: float = 0.0


# ── Cliente de streaming ──────────────────────────────────────────────────────

class TradierStreamClient:
    """WebSocket streaming de Tradier con reconexión automática.

    Internamente:
      1. Crea sesión HTTP → obtiene session_id
      2. Conecta WS con session_id
      3. Suscribe a símbolos
      4. Despacha eventos a callbacks en threads separados
    """

    LIVE_API_BASE    = "https://api.tradier.com/v1"
    SANDBOX_API_BASE = "https://sandbox.tradier.com/v1"
    STREAM_BASE      = "https://stream.tradier.com/v1"
    WS_STREAM_URL    = "wss://stream.tradier.com/v1/markets/events"

    def __init__(
        self,
        token: str,
        sandbox: bool = True,
        reconnect_delay_s: float = 3.0,
        max_reconnects: int = 20,
    ) -> None:
        self.token = token
        self.sandbox = sandbox
        self.reconnect_delay_s = reconnect_delay_s
        self.max_reconnects = max_reconnects

        self._api_base = self.SANDBOX_API_BASE if sandbox else self.LIVE_API_BASE
        self._symbols: list[str] = []
        self._session_id: str = ""
        self._ws: Optional[object] = None
        self._running = False
        self._reconnect_count = 0

        # Colas internas para despacho async
        self._quote_queue: queue.Queue[StreamQuote] = queue.Queue(maxsize=2000)
        self._trade_queue: queue.Queue[StreamTrade] = queue.Queue(maxsize=2000)

        # Callbacks
        self._quote_callbacks: list[Callable[[StreamQuote], None]] = []
        self._trade_callbacks: list[Callable[[StreamTrade], None]] = []
        self._error_callbacks: list[Callable[[str], None]] = []

        # Threads
        self._ws_thread: Optional[threading.Thread] = None
        self._dispatch_thread: Optional[threading.Thread] = None

    # ── Configuración ─────────────────────────────────────────────────────────

    def subscribe(self, symbols: list[str]) -> None:
        self._symbols = [s.upper() for s in symbols]

    def on_quote(self, cb: Callable[[StreamQuote], None]) -> None:
        self._quote_callbacks.append(cb)

    def on_trade(self, cb: Callable[[StreamTrade], None]) -> None:
        self._trade_callbacks.append(cb)

    def on_error(self, cb: Callable[[str], None]) -> None:
        self._error_callbacks.append(cb)

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Conecta en thread de fondo. No bloquea."""
        self._running = True
        self._ws_thread = threading.Thread(
            target=self._ws_loop, daemon=True, name="atlas-tradier-ws"
        )
        self._dispatch_thread = threading.Thread(
            target=self._dispatch_loop, daemon=True, name="atlas-tradier-dispatch"
        )
        self._ws_thread.start()
        self._dispatch_thread.start()
        logger.info("TradierStreamClient conectando (sandbox=%s)…", self.sandbox)

    def disconnect(self) -> None:
        self._running = False
        if self._ws:
            try:
                self._ws.close()  # type: ignore[union-attr]
            except Exception:
                pass
        logger.info("TradierStreamClient desconectado")

    # ── Sesión HTTP ───────────────────────────────────────────────────────────

    def _create_session(self) -> bool:
        """POST /markets/events/session → obtiene session_id."""
        url = f"{self._api_base}/markets/events/session"
        headers = {
            "Authorization": f"Bearer {self.token}",
            "Accept": "application/json",
        }
        try:
            r = requests.post(url, headers=headers, timeout=10)
            r.raise_for_status()
            data = r.json()
            self._session_id = data.get("stream", {}).get("sessionid", "")
            if not self._session_id:
                logger.error("No se obtuvo session_id: %s", data)
                return False
            logger.info("Sesión Tradier OK: %s…", self._session_id[:8])
            return True
        except Exception as exc:
            logger.error("Error creando sesión Tradier: %s", exc)
            return False

    # ── WebSocket loop ────────────────────────────────────────────────────────

    def _ws_loop(self) -> None:
        while self._running and self._reconnect_count < self.max_reconnects:
            if not self._create_session():
                time.sleep(self.reconnect_delay_s)
                self._reconnect_count += 1
                continue

            if not _WS_OK:
                logger.error("websocket-client no disponible")
                break

            ws = websocket.WebSocketApp(  # type: ignore[attr-defined]
                self.WS_STREAM_URL,
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_ws_error,
                on_close=self._on_close,
                header={"Authorization": f"Bearer {self.token}"},
            )
            self._ws = ws

            logger.info("Conectando WS Tradier (intento %d)…", self._reconnect_count + 1)
            ws.run_forever(ping_interval=25, ping_timeout=10)

            if not self._running:
                break

            self._reconnect_count += 1
            import random as _random
            jitter = _random.uniform(0.0, min(2.0, self.reconnect_delay_s * 0.3))
            delay  = self.reconnect_delay_s + jitter
            logger.warning(
                "WS desconectado — reintentando en %.1fs+%.1fs jitter (intento %d/%d)…",
                self.reconnect_delay_s, jitter, self._reconnect_count, self.max_reconnects
            )
            time.sleep(delay)

        if self._reconnect_count >= self.max_reconnects:
            logger.critical("Máximo de reconexiones alcanzado — stream detenido")
            for cb in self._error_callbacks:
                cb("max_reconnects_exceeded")

    def _on_open(self, ws: object) -> None:
        """Al conectar, suscribir a símbolos y tipos de evento."""
        self._reconnect_count = 0
        payload = {
            "symbols":    self._symbols,
            "sessionid":  self._session_id,
            "linebreak":  True,
            "filter":     ["quote", "trade", "timesale"],
        }
        ws.send(json.dumps(payload))  # type: ignore[attr-defined]
        logger.info("WS suscrito a %d símbolos", len(self._symbols))

    def _on_message(self, ws: object, raw: str) -> None:
        """Parsea evento Tradier y pone en colas internas."""
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        etype = data.get("type", "")
        ts = time.time()

        if etype == "quote":
            q = StreamQuote(
                symbol    = data.get("symbol", ""),
                bid       = float(data.get("bid", 0)),
                ask       = float(data.get("ask", 0)),
                bid_size  = int(data.get("bidsz", 0)),
                ask_size  = int(data.get("asksz", 0)),
                timestamp = ts,
            )
            self._quote_queue.put_nowait(q) if not self._quote_queue.full() else None

        elif etype in ("trade", "timesale"):
            price = float(data.get("price", data.get("last", 0)))
            size  = int(data.get("size", data.get("vol", 0)))
            t = StreamTrade(
                symbol    = data.get("symbol", ""),
                price     = price,
                size      = size,
                timestamp = ts,
                exchange  = data.get("exch", ""),
            )
            self._trade_queue.put_nowait(t) if not self._trade_queue.full() else None

    def _on_ws_error(self, ws: object, error: object) -> None:
        logger.error("WS error: %s", error)
        for cb in self._error_callbacks:
            cb(str(error))

    def _on_close(self, ws: object, code: object, msg: object) -> None:
        logger.warning("WS cerrado: code=%s msg=%s", code, msg)

    # ── Dispatch loop ─────────────────────────────────────────────────────────

    def _dispatch_loop(self) -> None:
        """Despacha eventos de las colas a los callbacks registrados."""
        while self._running:
            # Quotes
            while not self._quote_queue.empty():
                try:
                    q = self._quote_queue.get_nowait()
                    for cb in self._quote_callbacks:
                        cb(q)
                except Exception as exc:
                    logger.debug("Error dispatch quote: %s", exc)

            # Trades
            while not self._trade_queue.empty():
                try:
                    t = self._trade_queue.get_nowait()
                    for cb in self._trade_callbacks:
                        cb(t)
                except Exception as exc:
                    logger.debug("Error dispatch trade: %s", exc)

            time.sleep(0.01)

    # ── REST: Options Chain ───────────────────────────────────────────────────

    def get_options_chain(
        self, symbol: str, expiration: str | None = None
    ) -> Optional[OptionsChain]:
        """Obtiene chain de opciones para un símbolo vía REST."""
        url = f"{self._api_base}/markets/options/chains"
        headers = {
            "Authorization": f"Bearer {self.token}",
            "Accept": "application/json",
        }
        params: dict = {"symbol": symbol, "greeks": "true"}
        if expiration:
            params["expiration"] = expiration

        try:
            r = requests.get(url, headers=headers, params=params, timeout=10)
            r.raise_for_status()
            data = r.json()
            options = data.get("options", {}).get("option", [])
            if not options:
                return None
            return self._parse_chain(symbol, options)
        except Exception as exc:
            logger.error("Error obteniendo options chain %s: %s", symbol, exc)
            return None

    def _parse_chain(self, symbol: str, options: list[dict]) -> OptionsChain:
        """Parsea lista de opciones y computa métricas de IV."""
        calls = [o for o in options if o.get("option_type") == "call"]
        puts  = [o for o in options if o.get("option_type") == "put"]

        # IV promedio ATM (±2% del precio mid)
        mid_price = None
        atm_ivs: list[float] = []
        for o in options:
            iv = float(o.get("greeks", {}).get("mid_iv", 0) or 0)
            if iv > 0:
                atm_ivs.append(iv)

        atm_iv = float(sum(atm_ivs) / len(atm_ivs)) if atm_ivs else 0.0

        # Skew: diferencia IV put ATM vs call ATM
        call_ivs = [float(o.get("greeks", {}).get("mid_iv", 0) or 0) for o in calls if o.get("greeks")]
        put_ivs  = [float(o.get("greeks", {}).get("mid_iv", 0) or 0) for o in puts  if o.get("greeks")]
        put_call_skew = (
            (sum(put_ivs) / len(put_ivs)) - (sum(call_ivs) / len(call_ivs))
            if call_ivs and put_ivs else 0.0
        )

        return OptionsChain(
            symbol        = symbol,
            expiration    = options[0].get("expiration_date", "") if options else "",
            calls         = calls,
            puts          = puts,
            atm_iv        = atm_iv,
            put_call_skew = put_call_skew,
        )

    def get_quote(self, symbol: str) -> Optional[dict]:
        """Quote simple vía REST (para inicialización o fallback)."""
        url = f"{self._api_base}/markets/quotes"
        headers = {
            "Authorization": f"Bearer {self.token}",
            "Accept": "application/json",
        }
        try:
            r = requests.get(url, headers=headers, params={"symbols": symbol}, timeout=8)
            r.raise_for_status()
            data = r.json()
            q = data.get("quotes", {}).get("quote", {})
            return q if isinstance(q, dict) else None
        except Exception as exc:
            logger.error("Error quote REST %s: %s", symbol, exc)
            return None

    # ── Estadísticas ──────────────────────────────────────────────────────────

    def stats(self) -> dict:
        return {
            "running":         self._running,
            "symbols":         self._symbols,
            "reconnect_count": self._reconnect_count,
            "queue_quotes":    self._quote_queue.qsize(),
            "queue_trades":    self._trade_queue.qsize(),
            "sandbox":         self.sandbox,
        }
