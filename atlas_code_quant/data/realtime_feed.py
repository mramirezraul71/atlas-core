"""Atlas Code-Quant — Feed de datos en tiempo real via WebSocket.

Reemplaza el polling REST cada 180s por streams tick-by-tick.
Soporta dos fuentes:
  - Binance WebSocket (crypto) — gratis, sin auth para datos públicos
  - Alpaca Market Data WebSocket (acciones) — gratis con API key

El feed mantiene un buffer OHLCV actualizado en tiempo real que el
scanner y las estrategias pueden consumir sin latencia REST.

Grok/xAI recomendación: datos tick/1s de Binance para backtesting preciso
y ejecución autónoma 24/7 sin gaps.

Uso::
    feed = RealtimeFeed(source="binance")
    await feed.subscribe(["BTC/USDT", "ETH/USDT"], timeframe="1m")
    # En otro coroutine:
    df = feed.get_ohlcv("BTC/USDT", bars=200)
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Coroutine

import pandas as pd

logger = logging.getLogger("quant.data.realtime")

# ── Constantes ────────────────────────────────────────────────────────────────
_BINANCE_WS_BASE   = "wss://stream.binance.com:9443/stream"
_ALPACA_WS_DATA    = "wss://stream.data.alpaca.markets/v2"
_RECONNECT_BASE_S  = 2.0    # Backoff inicial
_RECONNECT_MAX_S   = 60.0   # Backoff máximo
_BAR_BUFFER_SIZE   = 500    # Máximo de barras en memoria por símbolo


@dataclass
class OHLCVBar:
    """Vela OHLCV en tiempo real."""
    timestamp: datetime
    open: float
    high: float
    low: float
    close: float
    volume: float
    is_closed: bool = False     # True = vela cerrada, False = vela en formación


@dataclass
class FeedStats:
    """Estadísticas del feed para monitoreo."""
    symbol: str
    last_update: float = 0.0
    bars_received: int = 0
    reconnects: int = 0
    last_price: float = 0.0
    latency_ms: float = 0.0


class RealtimeFeed:
    """Feed de datos de mercado en tiempo real via WebSocket.

    Mantiene un buffer OHLCV actualizado por símbolo y timeframe.
    Los consumidores (scanner, estrategias) llaman a `get_ohlcv()` para
    obtener las últimas N barras como DataFrame — igual interfaz que MarketFeed.

    Args:
        source: "binance" (crypto) o "alpaca" (acciones US).
        alpaca_api_key: API key de Alpaca (solo si source="alpaca").
        alpaca_secret_key: Secret key de Alpaca.
        feed_type: "iex" o "sip" (solo Alpaca; default "iex" = gratis).
        bar_buffer_size: Máximo de barras en memoria por símbolo.

    Example::
        feed = RealtimeFeed(source="binance")
        await feed.subscribe(["BTC/USDT", "ETH/USDT"], timeframe="1m")
        df = feed.get_ohlcv("BTC/USDT", bars=100)
        stats = feed.stats("BTC/USDT")
    """

    def __init__(
        self,
        source: str = "binance",
        alpaca_api_key: str = "",
        alpaca_secret_key: str = "",
        feed_type: str = "iex",
        bar_buffer_size: int = _BAR_BUFFER_SIZE,
    ) -> None:
        self.source = source.lower()
        self._alpaca_key    = alpaca_api_key
        self._alpaca_secret = alpaca_secret_key
        self._feed_type     = feed_type
        self._bar_size      = bar_buffer_size

        # symbol → timeframe → deque de OHLCVBar
        self._bars: dict[str, dict[str, deque]] = defaultdict(lambda: defaultdict(lambda: deque(maxlen=bar_buffer_size)))
        # symbol → OHLCVBar en formación (vela actual no cerrada)
        self._current_bar: dict[str, dict[str, OHLCVBar | None]] = defaultdict(lambda: defaultdict(lambda: None))
        # Estadísticas por símbolo
        self._stats: dict[str, FeedStats] = {}
        # Callbacks: fn(symbol, bar) llamado en cada vela cerrada
        self._on_bar_closed: list[Callable] = []
        # Symbols activos
        self._subscribed_symbols: set[str] = set()
        self._subscribed_timeframe: str = "1m"
        # Task de fondo
        self._ws_task: asyncio.Task | None = None
        self._running = False

    # ── API Pública ───────────────────────────────────────────────────────────

    def get_ohlcv(self, symbol: str, timeframe: str = "1m", bars: int = 200) -> pd.DataFrame:
        """Retorna las últimas N barras como DataFrame OHLCV.

        Misma interfaz que MarketFeed.ohlcv() para compatibilidad con
        estrategias y scanner existentes.

        Args:
            symbol: Ticker (ej: "BTC/USDT" o "AAPL").
            timeframe: Marco temporal suscrito.
            bars: Número de barras a retornar.

        Returns:
            DataFrame con columnas [open, high, low, close, volume], index timestamp UTC.
            DataFrame vacío si no hay datos aún.
        """
        buf = self._bars[symbol][timeframe]
        if not buf:
            return pd.DataFrame(columns=["open", "high", "low", "close", "volume"])

        records = list(buf)[-bars:]
        df = pd.DataFrame([{
            "timestamp": b.timestamp,
            "open":   b.open,
            "high":   b.high,
            "low":    b.low,
            "close":  b.close,
            "volume": b.volume,
        } for b in records])
        df.set_index("timestamp", inplace=True)
        df.index = pd.to_datetime(df.index, utc=True)
        return df.astype(float)

    def get_last_price(self, symbol: str) -> float:
        """Retorna el último precio conocido del símbolo."""
        stats = self._stats.get(symbol)
        return stats.last_price if stats else 0.0

    def is_live(self, symbol: str, max_stale_s: float = 30.0) -> bool:
        """Retorna True si el símbolo recibió datos en los últimos max_stale_s segundos."""
        stats = self._stats.get(symbol)
        if not stats:
            return False
        return (time.time() - stats.last_update) < max_stale_s

    def stats(self, symbol: str) -> dict:
        """Retorna estadísticas del feed para el símbolo dado."""
        s = self._stats.get(symbol, FeedStats(symbol))
        return {
            "symbol":         s.symbol,
            "last_update_ts": datetime.fromtimestamp(s.last_update, tz=timezone.utc).isoformat() if s.last_update else None,
            "bars_received":  s.bars_received,
            "reconnects":     s.reconnects,
            "last_price":     s.last_price,
            "latency_ms":     round(s.latency_ms, 2),
            "is_live":        self.is_live(symbol),
        }

    def all_stats(self) -> list[dict]:
        return [self.stats(s) for s in self._subscribed_symbols]

    def on_bar_closed(self, callback: Callable) -> None:
        """Registra callback llamado cuando se cierra una barra: fn(symbol, bar: OHLCVBar)."""
        self._on_bar_closed.append(callback)

    async def subscribe(
        self,
        symbols: list[str],
        timeframe: str = "1m",
    ) -> None:
        """Suscribe al feed en tiempo real para los símbolos dados.

        Lanza la tarea WebSocket en background. No bloquea.

        Args:
            symbols: Lista de tickers.
            timeframe: Marco temporal para las barras (ej: "1m", "5m").
        """
        self._subscribed_symbols = set(symbols)
        self._subscribed_timeframe = timeframe
        for s in symbols:
            self._stats[s] = FeedStats(symbol=s)
        self._running = True
        self._ws_task = asyncio.create_task(self._run_ws_loop(symbols, timeframe))
        logger.info("[RealtimeFeed] Suscrito a %d símbolos via %s (%s)",
                    len(symbols), self.source, timeframe)

    async def stop(self) -> None:
        """Detiene el feed y cancela la tarea WebSocket."""
        self._running = False
        if self._ws_task and not self._ws_task.done():
            self._ws_task.cancel()
            try:
                await self._ws_task
            except asyncio.CancelledError:
                pass
        logger.info("[RealtimeFeed] Feed detenido")

    # ── Loop WebSocket con reconexión automática ──────────────────────────────

    async def _run_ws_loop(self, symbols: list[str], timeframe: str) -> None:
        """Loop principal con backoff exponencial en reconexión."""
        delay = _RECONNECT_BASE_S
        while self._running:
            try:
                if self.source == "binance":
                    await self._run_binance(symbols, timeframe)
                elif self.source == "alpaca":
                    await self._run_alpaca(symbols, timeframe)
                else:
                    logger.error("[RealtimeFeed] Fuente desconocida: %s", self.source)
                    break
                delay = _RECONNECT_BASE_S  # Reset backoff en conexión exitosa
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.warning("[RealtimeFeed] Desconexión — reconectando en %.1fs: %s", delay, exc)
                for s in symbols:
                    if s in self._stats:
                        self._stats[s].reconnects += 1
                if self._running:
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, _RECONNECT_MAX_S)

    # ── Binance WebSocket ─────────────────────────────────────────────────────

    async def _run_binance(self, symbols: list[str], timeframe: str) -> None:
        """Conecta al stream de klines de Binance para todos los símbolos."""
        try:
            import websockets
        except ImportError:
            logger.error("[RealtimeFeed] 'websockets' no instalado — pip install websockets")
            return

        tf_map = {"1m": "1m", "3m": "3m", "5m": "5m", "15m": "15m",
                  "30m": "30m", "1h": "1h", "4h": "4h", "1d": "1d"}
        binance_tf = tf_map.get(timeframe, "1m")

        # Normalizar símbolos: BTC/USDT → btcusdt
        streams = []
        sym_map = {}  # stream_name → symbol original
        for sym in symbols:
            clean = sym.replace("/", "").lower()
            stream = f"{clean}@kline_{binance_tf}"
            streams.append(stream)
            sym_map[clean] = sym

        url = f"{_BINANCE_WS_BASE}?streams={'/'.join(streams)}"
        logger.info("[Binance WS] Conectando: %d streams", len(streams))

        async with websockets.connect(url, ping_interval=20, ping_timeout=10) as ws:
            logger.info("[Binance WS] Conectado ✓")
            async for raw_msg in ws:
                if not self._running:
                    break
                try:
                    msg = json.loads(raw_msg)
                    data = msg.get("data", msg)
                    if data.get("e") != "kline":
                        continue
                    k = data["k"]
                    clean_sym = data["s"].lower()
                    original_sym = sym_map.get(clean_sym, data["s"])
                    self._process_binance_kline(original_sym, k, timeframe)
                except Exception as exc:
                    logger.debug("[Binance WS] Error procesando mensaje: %s", exc)

    def _process_binance_kline(self, symbol: str, k: dict, timeframe: str) -> None:
        """Procesa una vela kline de Binance y actualiza el buffer."""
        ts = datetime.fromtimestamp(k["t"] / 1000, tz=timezone.utc)
        bar = OHLCVBar(
            timestamp=ts,
            open=float(k["o"]),
            high=float(k["h"]),
            low=float(k["l"]),
            close=float(k["c"]),
            volume=float(k["v"]),
            is_closed=bool(k["x"]),  # True = vela cerrada
        )

        now = time.time()
        if symbol not in self._stats:
            self._stats[symbol] = FeedStats(symbol=symbol)
        self._stats[symbol].last_update = now
        self._stats[symbol].last_price  = bar.close
        self._stats[symbol].latency_ms  = (now - k["T"] / 1000) * 1000

        if bar.is_closed:
            self._bars[symbol][timeframe].append(bar)
            self._stats[symbol].bars_received += 1
            self._fire_on_bar_closed(symbol, bar)
        else:
            # Actualizar vela en formación (no agregar al buffer todavía)
            self._current_bar[symbol][timeframe] = bar

    # ── Alpaca WebSocket ──────────────────────────────────────────────────────

    async def _run_alpaca(self, symbols: list[str], timeframe: str) -> None:
        """Conecta al feed de barras de Alpaca para acciones US."""
        try:
            import websockets
        except ImportError:
            logger.error("[RealtimeFeed] 'websockets' no instalado — pip install websockets")
            return

        if not self._alpaca_key or not self._alpaca_secret:
            logger.error("[RealtimeFeed] Alpaca requiere API key y secret — configurar ALPACA_API_KEY/ALPACA_SECRET_KEY")
            return

        url = f"{_ALPACA_WS_DATA}/{self._feed_type}"
        logger.info("[Alpaca WS] Conectando a %s", url)

        async with websockets.connect(url, ping_interval=20) as ws:
            # Autenticación
            await ws.send(json.dumps({
                "action": "auth",
                "key":    self._alpaca_key,
                "secret": self._alpaca_secret,
            }))
            auth_resp = json.loads(await ws.recv())
            if not any(m.get("T") == "success" for m in auth_resp):
                logger.error("[Alpaca WS] Autenticación fallida: %s", auth_resp)
                return
            logger.info("[Alpaca WS] Autenticado ✓")

            # Suscripción a barras del timeframe
            # Alpaca soporta barras 1Min, 5Min, etc.
            await ws.send(json.dumps({
                "action": "subscribe",
                "bars":   symbols,
            }))
            logger.info("[Alpaca WS] Suscrito a %d símbolos ✓", len(symbols))

            async for raw_msg in ws:
                if not self._running:
                    break
                try:
                    msgs = json.loads(raw_msg)
                    for msg in msgs:
                        if msg.get("T") == "b":  # bar
                            self._process_alpaca_bar(msg, timeframe)
                except Exception as exc:
                    logger.debug("[Alpaca WS] Error procesando mensaje: %s", exc)

    def _process_alpaca_bar(self, msg: dict, timeframe: str) -> None:
        """Procesa una barra de Alpaca y actualiza el buffer."""
        symbol = msg.get("S", "")
        ts_str = msg.get("t", "")
        try:
            ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00"))
        except Exception:
            ts = datetime.now(tz=timezone.utc)

        bar = OHLCVBar(
            timestamp=ts,
            open=float(msg.get("o", 0)),
            high=float(msg.get("h", 0)),
            low=float(msg.get("l", 0)),
            close=float(msg.get("c", 0)),
            volume=float(msg.get("v", 0)),
            is_closed=True,  # Alpaca solo envía barras cerradas
        )

        now = time.time()
        if symbol not in self._stats:
            self._stats[symbol] = FeedStats(symbol=symbol)
        self._stats[symbol].last_update = now
        self._stats[symbol].last_price  = bar.close
        self._stats[symbol].bars_received += 1
        self._bars[symbol][timeframe].append(bar)
        self._fire_on_bar_closed(symbol, bar)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _fire_on_bar_closed(self, symbol: str, bar: OHLCVBar) -> None:
        for cb in self._on_bar_closed:
            try:
                if asyncio.iscoroutinefunction(cb):
                    asyncio.create_task(cb(symbol, bar))
                else:
                    cb(symbol, bar)
            except Exception as exc:
                logger.debug("[RealtimeFeed] Callback error: %s", exc)


# ── Singleton global ──────────────────────────────────────────────────────────
_feed_instance: RealtimeFeed | None = None


def get_realtime_feed(
    source: str = "binance",
    alpaca_api_key: str = "",
    alpaca_secret_key: str = "",
) -> RealtimeFeed:
    """Retorna la instancia global del RealtimeFeed (lazy singleton)."""
    global _feed_instance
    if _feed_instance is None:
        _feed_instance = RealtimeFeed(
            source=source,
            alpaca_api_key=alpaca_api_key,
            alpaca_secret_key=alpaca_secret_key,
        )
    return _feed_instance
