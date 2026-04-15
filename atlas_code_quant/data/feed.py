"""Atlas Code-Quant — Ingestión de datos de mercado (OHLCV + orderbook).

Soporta: ccxt (crypto) y yfinance (acciones).
"""
from __future__ import annotations

import logging
from datetime import datetime
from typing import Literal

import pandas as pd

logger = logging.getLogger("quant.data.feed")


def _iter_chunks(items: list[str], size: int):
    for idx in range(0, len(items), size):
        yield items[idx : idx + size]


def _normalize_yfinance_columns(df: pd.DataFrame) -> pd.DataFrame:
    """Aplana columnas MultiIndex y normaliza nombres OHLCV."""
    if isinstance(df.columns, pd.MultiIndex):
        flat_cols: list[str] = []
        for col in df.columns:
            names = [str(part).lower() for part in col if part not in ("", None)]
            flat_cols.append(names[0] if names else "")
        df.columns = flat_cols
    else:
        df.columns = [str(col).lower() for col in df.columns]

    rename_map = {
        "adj close": "adj_close",
    }
    df = df.rename(columns=rename_map)
    required = [col for col in ("open", "high", "low", "close", "volume") if col in df.columns]
    return df[required].copy() if required else df.copy()


def _resample_ohlcv(df: pd.DataFrame, rule: str) -> pd.DataFrame:
    """Re-muestrea OHLCV a una temporalidad mayor preservando volumen."""
    if df.empty:
        return df
    out = pd.DataFrame({
        "open": df["open"].resample(rule).first(),
        "high": df["high"].resample(rule).max(),
        "low": df["low"].resample(rule).min(),
        "close": df["close"].resample(rule).last(),
        "volume": df["volume"].resample(rule).sum() if "volume" in df.columns else 0.0,
    }).dropna(subset=["open", "high", "low", "close"])
    return out


class MarketFeed:
    """Abstracción unificada de datos para crypto y acciones.

    Args:
        source: "ccxt" para crypto, "yfinance" para acciones.
        exchange_id: ID del exchange ccxt (ej: "binance"). Solo para ccxt.
        sandbox: Usar entorno de prueba del exchange.

    Example::
        feed = MarketFeed(source="ccxt", exchange_id="binance")
        df = feed.ohlcv("BTC/USDT", "1h", limit=200)
    """

    def __init__(
        self,
        source: Literal["ccxt", "yfinance"] = "ccxt",
        exchange_id: str = "binance",
        sandbox: bool = True,
    ) -> None:
        self.source = source
        self._exchange = None
        if source == "ccxt":
            self._exchange = self._init_ccxt(exchange_id, sandbox)

    def _init_ccxt(self, exchange_id: str, sandbox: bool):
        import ccxt
        exchange_cls = getattr(ccxt, exchange_id)
        ex = exchange_cls({"enableRateLimit": True})
        if sandbox:
            ex.set_sandbox_mode(True)
        return ex

    def ohlcv(
        self,
        symbol: str,
        timeframe: str = "1h",
        limit: int = 500,
        since: datetime | None = None,
    ) -> pd.DataFrame:
        """Descarga velas OHLCV.

        Args:
            symbol: Par de trading (ej: "BTC/USDT") o ticker (ej: "AAPL").
            timeframe: Marco temporal ("1m","5m","1h","4h","1d").
            limit: Número de velas a retornar.
            since: Timestamp inicial (opcional).

        Returns:
            DataFrame con columnas [timestamp, open, high, low, close, volume].
        """
        if self.source == "ccxt":
            return self._ohlcv_ccxt(symbol, timeframe, limit, since)
        return self._ohlcv_yfinance(symbol, timeframe, limit, since)

    def ohlcv_many(
        self,
        symbols: list[str],
        timeframe: str = "1d",
        limit: int = 120,
    ) -> dict[str, pd.DataFrame]:
        """Descarga OHLCV para varios simbolos.

        En yfinance usa descarga agrupada; en ccxt cae a bucle simple.
        """
        unique_symbols = [str(symbol).strip().upper() for symbol in symbols if str(symbol).strip()]
        unique_symbols = list(dict.fromkeys(unique_symbols))
        if not unique_symbols:
            return {}
        if self.source == "ccxt":
            return {symbol: self._ohlcv_ccxt(symbol, timeframe, limit, None) for symbol in unique_symbols}
        return self._ohlcv_many_yfinance(unique_symbols, timeframe=timeframe, limit=limit)

    def _ohlcv_ccxt(self, symbol, timeframe, limit, since):
        since_ms = int(since.timestamp() * 1000) if since else None
        raw = self._exchange.fetch_ohlcv(symbol, timeframe, since=since_ms, limit=limit)
        df = pd.DataFrame(raw, columns=["timestamp", "open", "high", "low", "close", "volume"])
        df["timestamp"] = pd.to_datetime(df["timestamp"], unit="ms", utc=True)
        df.set_index("timestamp", inplace=True)
        return df.astype(float)

    def _ohlcv_yfinance(self, symbol, timeframe, limit, since):
        import yfinance as yf
        interval, period = self._yfinance_interval_period(timeframe)
        df = yf.download(
            symbol,
            period=period,
            interval=interval,
            progress=False,
            auto_adjust=False,
            threads=False,
            timeout=10,  # Evitar bloqueo indefinido con símbolos delisted
        )
        df = _normalize_yfinance_columns(df)
        if df.empty:
            return df
        df.index.name = "timestamp"
        if df.index.tz is None:
            df.index = pd.to_datetime(df.index, utc=True)
        else:
            df.index = pd.to_datetime(df.index, utc=True)
        if timeframe == "4h":
            df = _resample_ohlcv(df, "4h")
        return df.tail(limit)

    def _ohlcv_many_yfinance(self, symbols: list[str], timeframe: str, limit: int) -> dict[str, pd.DataFrame]:
        import yfinance as yf

        interval, period = self._yfinance_interval_period(timeframe)
        out: dict[str, pd.DataFrame] = {}
        chunk_size = 20 if timeframe == "1d" else 10
        # Limita fallback individual para evitar ciclos del scanner bloqueados
        # cuando Yahoo no responde para muchos símbolos del batch.
        max_symbol_fallback = 4
        for chunk in _iter_chunks(symbols, chunk_size):
            try:
                raw = yf.download(
                    chunk,
                    period=period,
                    interval=interval,
                    progress=False,
                    auto_adjust=False,
                    group_by="ticker",
                    threads=False,
                    timeout=15,  # Timeout batch: 15s máximo
                )
                out.update(self._extract_yfinance_batch_frames(raw, chunk, timeframe, limit))
            except Exception as exc:
                logger.warning("Fallo descarga agrupada de %s simbolos: %s", len(chunk), exc)
            missing = [symbol for symbol in chunk if symbol not in out]
            if len(missing) > max_symbol_fallback:
                logger.warning(
                    "Fallback individual truncado (%s/%s símbolos) para evitar bloqueo",
                    max_symbol_fallback,
                    len(missing),
                )
                missing = missing[:max_symbol_fallback]
            for symbol in missing:
                try:
                    out[symbol] = self._ohlcv_yfinance(symbol, timeframe, limit, None)
                except Exception:
                    continue
        return out

    def _extract_yfinance_batch_frames(
        self,
        raw: pd.DataFrame,
        symbols: list[str],
        timeframe: str,
        limit: int,
    ) -> dict[str, pd.DataFrame]:
        out: dict[str, pd.DataFrame] = {}
        if raw is None or raw.empty:
            return out
        if len(symbols) == 1:
            symbol = symbols[0]
            single_raw = raw[symbol].copy() if isinstance(raw.columns, pd.MultiIndex) and symbol in set(raw.columns.get_level_values(0)) else raw
            df = _normalize_yfinance_columns(single_raw)
            if not df.empty:
                df.index.name = "timestamp"
                df.index = pd.to_datetime(df.index, utc=True)
                if timeframe == "4h":
                    df = _resample_ohlcv(df, "4h")
                out[symbol] = df.tail(limit)
            return out
        if not isinstance(raw.columns, pd.MultiIndex):
            return out
        level0 = set(raw.columns.get_level_values(0))
        for symbol in symbols:
            if symbol not in level0:
                continue
            df = _normalize_yfinance_columns(raw[symbol].copy())
            if df.empty:
                continue
            df.index.name = "timestamp"
            df.index = pd.to_datetime(df.index, utc=True)
            if timeframe == "4h":
                df = _resample_ohlcv(df, "4h")
            out[symbol] = df.tail(limit)
        return out

    def _yfinance_interval_period(self, timeframe: str) -> tuple[str, str]:
        tf_map = {
            "1m": "1m",
            "5m": "5m",
            "15m": "15m",
            "30m": "30m",
            "1h": "60m",
            "4h": "60m",
            "1d": "1d",
        }
        period_map = {
            "1m": "7d",
            "5m": "60d",
            "15m": "60d",
            "30m": "60d",
            "1h": "730d",
            "4h": "730d",
            "1d": "5y",
        }
        return tf_map.get(timeframe, "1d"), period_map.get(timeframe, "1y")

    def ticker(self, symbol: str) -> dict:
        """Retorna precio actual y estadísticas 24h.

        Returns:
            dict con keys: last, bid, ask, volume, change_pct.
        """
        if self.source == "ccxt":
            t = self._exchange.fetch_ticker(symbol)
            return {
                "last": t["last"],
                "bid": t["bid"],
                "ask": t["ask"],
                "volume": t["quoteVolume"],
                "change_pct": t["percentage"],
            }
        raise NotImplementedError("ticker() solo disponible con ccxt por ahora")
