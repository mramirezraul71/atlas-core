"""Módulo 2B — Indicadores Técnicos + CVD + IV Rank.

Calcula en tiempo real:
  - CVD (Cumulative Volume Delta): absorción + desequilibrio de delta
  - IV Rank (percentil 30 días) + IV/HV Ratio
  - ADX(14), RSI(14), MACD, ATR(20)
  - Exponente de Hurst (memoria de serie temporal)
  - Slope del CVD (tendencia de presión compradora/vendedora)

Diseñado para bajo consumo en Jetson: usa numpy puro, sin pandas en hot-path.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger("atlas.pipeline.indicators")


# ── Estructuras ───────────────────────────────────────────────────────────────

@dataclass
class CVDSnapshot:
    """Estado del CVD en un instante."""
    cumulative_delta: float = 0.0   # positivo = presión compradora neta
    delta_imbalance: float = 0.0    # last_delta / avg_volume
    absorption: float = 0.0         # cuánto volumen fue absorbido sin mover precio
    slope_1m: float = 0.0           # pendiente lineal última 1 min
    slope_5m: float = 0.0           # pendiente lineal últimos 5 min
    timestamp: float = field(default_factory=time.time)


@dataclass
class IVMetrics:
    """Métricas de Volatilidad Implícita."""
    symbol: str = ""
    current_iv: float = 0.0
    iv_rank_30d: float = 0.0        # percentil IV actual vs 30 días
    iv_hv_ratio: float = 0.0        # IV / HV(20) — >1.2 → sobreestimada
    hv_20: float = 0.0              # Historical Volatility 20 barras
    atm_skew: float = 0.0           # put_iv - call_iv ATM
    timestamp: float = field(default_factory=time.time)


@dataclass
class TechnicalSnapshot:
    """Snapshot de indicadores técnicos clásicos."""
    symbol: str = ""
    close: float = 0.0
    rsi_14: float = 50.0
    macd: float = 0.0
    macd_signal: float = 0.0
    macd_hist: float = 0.0
    adx_14: float = 0.0
    atr_20: float = 0.0
    volume_ratio: float = 1.0       # vol actual / vol promedio 20 barras
    hurst: float = 0.5              # 0.5=random, >0.5=tendencia, <0.5=reversión
    timestamp: float = field(default_factory=time.time)

    @property
    def rsi_divergence(self) -> bool:
        """Señal de divergencia RSI (necesita historial externo)."""
        return self.rsi_14 < 30 or self.rsi_14 > 70

    @property
    def volume_spike(self) -> bool:
        """Pico de volumen >1.8× promedio."""
        return self.volume_ratio > 1.8


# ── CVD Calculator ────────────────────────────────────────────────────────────

class CVDCalculator:
    """Calcula CVD acumulado desde stream de trades (Lee-Ready simplificado).

    Lee-Ready rule:
      - trade.price > (bid+ask)/2  → BUY (agressor)
      - trade.price < (bid+ask)/2  → SELL (agressor)
      - trade.price == mid          → anterior

    Mantiene ventana deslizante de 5 minutos para slope.
    """

    SLOPE_WINDOW_S = {
        "1m": 60,
        "5m": 300,
    }

    def __init__(self, window_trades: int = 5000) -> None:
        self._trades: deque[dict] = deque(maxlen=window_trades)
        self._cumulative_delta: float = 0.0
        self._last_mid: float = 0.0
        self._last_bid: float = 0.0
        self._last_ask: float = 0.0

        # Historial (timestamp, delta_at_point) para slope
        self._delta_history: deque[tuple[float, float]] = deque(maxlen=10000)

    def update_quote(self, bid: float, ask: float) -> None:
        self._last_bid = bid
        self._last_ask = ask
        self._last_mid = (bid + ask) / 2

    def update_trade(self, price: float, size: int, timestamp: float | None = None) -> float:
        """Procesa un trade y retorna el delta del trade."""
        ts = timestamp or time.time()
        mid = self._last_mid or price

        # Lee-Ready
        if price > mid:
            delta = float(size)
        elif price < mid:
            delta = -float(size)
        else:
            delta = 0.0

        self._cumulative_delta += delta
        self._trades.append({"ts": ts, "price": price, "size": size, "delta": delta})
        self._delta_history.append((ts, self._cumulative_delta))
        return delta

    def snapshot(self) -> CVDSnapshot:
        now = time.time()
        slope_1m = self._compute_slope(now - 60)
        slope_5m = self._compute_slope(now - 300)

        # Delta imbalance: delta del último minuto / volumen total último minuto
        recent = [t for t in self._trades if t["ts"] > now - 60]
        total_vol = sum(t["size"] for t in recent)
        recent_delta = sum(t["delta"] for t in recent)
        imbalance = recent_delta / total_vol if total_vol > 0 else 0.0

        # Absorción: volumen sin mover precio significativamente
        absorption = self._compute_absorption(recent)

        return CVDSnapshot(
            cumulative_delta = self._cumulative_delta,
            delta_imbalance  = imbalance,
            absorption       = absorption,
            slope_1m         = slope_1m,
            slope_5m         = slope_5m,
            timestamp        = now,
        )

    def _compute_slope(self, since: float) -> float:
        """Regresión lineal del CVD desde `since`."""
        pts = [(ts, d) for ts, d in self._delta_history if ts >= since]
        if len(pts) < 2:
            return 0.0
        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        xs -= xs[0]
        if xs[-1] == 0:
            return 0.0
        slope = float(np.polyfit(xs, ys, 1)[0])
        return slope

    def _compute_absorption(self, recent_trades: list[dict]) -> float:
        """Cuánto volumen no movió el precio (absorción de liquidez)."""
        if len(recent_trades) < 2:
            return 0.0
        prices = [t["price"] for t in recent_trades]
        price_range = max(prices) - min(prices)
        if price_range < 1e-8:
            # Precio plano pese a volumen → alta absorción
            total_vol = sum(t["size"] for t in recent_trades)
            return float(total_vol)
        return 0.0

    def reset(self) -> None:
        self._cumulative_delta = 0.0
        self._trades.clear()
        self._delta_history.clear()


# ── IV Rank Calculator ────────────────────────────────────────────────────────

class IVRankCalculator:
    """Mantiene historial de IV para calcular IV Rank (percentil 30d).

    El IV Rank (IVR) se calcula como:
        IVR = (IV_actual - IV_min_30d) / (IV_max_30d - IV_min_30d) × 100

    Diferente del IV Percentile (que usa % de días por debajo).
    """

    HISTORY_DAYS = 30
    MAX_SAMPLES = HISTORY_DAYS * 390   # 390 min/día mercado

    def __init__(self) -> None:
        self._iv_history: dict[str, deque[tuple[float, float]]] = {}  # symbol → [(ts, iv)]
        self._hv_windows: dict[str, deque[float]] = {}                 # symbol → returns

    def update_iv(self, symbol: str, iv: float, timestamp: float | None = None) -> None:
        ts = timestamp or time.time()
        if symbol not in self._iv_history:
            self._iv_history[symbol] = deque(maxlen=self.MAX_SAMPLES)
        self._iv_history[symbol].append((ts, iv))

    def update_close(self, symbol: str, close: float) -> None:
        """Registra cierre para cálculo de HV."""
        if symbol not in self._hv_windows:
            self._hv_windows[symbol] = deque(maxlen=21)
        self._hv_windows[symbol].append(close)

    def metrics(self, symbol: str, current_iv: float) -> IVMetrics:
        self.update_iv(symbol, current_iv)
        m = IVMetrics(symbol=symbol, current_iv=current_iv)

        # IV Rank 30d
        cutoff = time.time() - self.HISTORY_DAYS * 86400
        hist = [iv for ts, iv in self._iv_history.get(symbol, []) if ts >= cutoff]
        if len(hist) >= 5:
            iv_min, iv_max = min(hist), max(hist)
            denom = iv_max - iv_min
            m.iv_rank_30d = ((current_iv - iv_min) / denom * 100) if denom > 1e-8 else 50.0

        # HV(20) y IV/HV ratio
        closes = list(self._hv_windows.get(symbol, []))
        if len(closes) >= 2:
            returns = np.diff(np.log(np.array(closes, dtype=float) + 1e-10))
            m.hv_20 = float(np.std(returns) * np.sqrt(252) * 100)
            if m.hv_20 > 0:
                m.iv_hv_ratio = current_iv / m.hv_20

        return m


# ── Indicadores Técnicos ──────────────────────────────────────────────────────

class TechnicalIndicators:
    """Calcula RSI, MACD, ADX, ATR, Hurst sobre ventanas deslizantes.

    Uso::

        ti = TechnicalIndicators("AAPL")
        for bar in bars:
            snap = ti.update(bar["close"], bar["high"], bar["low"], bar["volume"])
            print(snap.rsi_14, snap.adx_14)
    """

    def __init__(self, symbol: str, max_history: int = 500) -> None:
        self.symbol = symbol
        self._closes:  deque[float] = deque(maxlen=max_history)
        self._highs:   deque[float] = deque(maxlen=max_history)
        self._lows:    deque[float] = deque(maxlen=max_history)
        self._volumes: deque[float] = deque(maxlen=max_history)

        # EMA para MACD
        self._ema12: Optional[float] = None
        self._ema26: Optional[float] = None
        self._ema_signal: Optional[float] = None

        # Wilder smoothing para RSI
        self._avg_gain: Optional[float] = None
        self._avg_loss: Optional[float] = None

        # Wilder para ADX
        self._adx_smooth: Optional[float] = None
        self._plus_di_smooth: Optional[float] = None
        self._minus_di_smooth: Optional[float] = None

    def update(
        self,
        close: float,
        high: float,
        low: float,
        volume: float = 0.0,
    ) -> TechnicalSnapshot:
        """Actualiza con nueva barra y retorna snapshot de indicadores."""
        self._closes.append(close)
        self._highs.append(high)
        self._lows.append(low)
        self._volumes.append(volume)

        snap = TechnicalSnapshot(symbol=self.symbol, close=close)
        n = len(self._closes)

        if n < 2:
            return snap

        closes = np.array(self._closes, dtype=float)

        snap.rsi_14    = self._rsi(closes)
        snap.macd, snap.macd_signal, snap.macd_hist = self._macd(closes)
        snap.adx_14    = self._adx()
        snap.atr_20    = self._atr(20)
        snap.volume_ratio = self._vol_ratio()

        if n >= 100:
            snap.hurst = self._hurst(closes[-100:])

        return snap

    # ── RSI (Wilder smoothing) ────────────────────────────────────────────────

    def _rsi(self, closes: np.ndarray, period: int = 14) -> float:
        if len(closes) < period + 1:
            return 50.0
        deltas = np.diff(closes)
        gains = np.where(deltas > 0, deltas, 0.0)
        losses = np.where(deltas < 0, -deltas, 0.0)

        if self._avg_gain is None:
            self._avg_gain = float(gains[-period:].mean())
            self._avg_loss = float(losses[-period:].mean())
        else:
            g = gains[-1]
            l = losses[-1]
            self._avg_gain = (self._avg_gain * (period - 1) + g) / period
            self._avg_loss = (self._avg_loss * (period - 1) + l) / period

        if self._avg_loss < 1e-10:
            return 100.0
        rs = self._avg_gain / self._avg_loss
        return float(100 - 100 / (1 + rs))

    # ── MACD ──────────────────────────────────────────────────────────────────

    def _macd(self, closes: np.ndarray) -> tuple[float, float, float]:
        c = closes[-1]
        k12, k26, k9 = 2/(12+1), 2/(26+1), 2/(9+1)

        if self._ema12 is None:
            self._ema12 = float(closes[:12].mean()) if len(closes) >= 12 else c
        if self._ema26 is None:
            self._ema26 = float(closes[:26].mean()) if len(closes) >= 26 else c

        self._ema12 = c * k12 + self._ema12 * (1 - k12)
        self._ema26 = c * k26 + self._ema26 * (1 - k26)
        macd_val = self._ema12 - self._ema26

        if self._ema_signal is None:
            self._ema_signal = macd_val
        self._ema_signal = macd_val * k9 + self._ema_signal * (1 - k9)

        hist = macd_val - self._ema_signal
        return float(macd_val), float(self._ema_signal), float(hist)

    # ── ADX ───────────────────────────────────────────────────────────────────

    def _adx(self, period: int = 14) -> float:
        n = len(self._closes)
        if n < period + 2:
            return 0.0

        highs  = np.array(self._highs,  dtype=float)
        lows   = np.array(self._lows,   dtype=float)
        closes = np.array(self._closes, dtype=float)

        # True Range
        tr_arr = np.maximum(highs[1:] - lows[1:],
                 np.maximum(np.abs(highs[1:] - closes[:-1]),
                            np.abs(lows[1:]  - closes[:-1])))

        dmp = np.maximum(highs[1:] - highs[:-1], 0)
        dmm = np.maximum(lows[:-1] - lows[1:], 0)
        dmp = np.where(dmp > dmm, dmp, 0.0)
        dmm = np.where(dmm >= dmp, dmm, 0.0)

        if len(tr_arr) < period:
            return 0.0

        atr  = tr_arr[-period:].mean()
        if atr < 1e-10:
            return 0.0

        plus_di  = 100 * dmp[-period:].mean() / atr
        minus_di = 100 * dmm[-period:].mean() / atr
        denom = plus_di + minus_di
        if denom < 1e-10:
            return 0.0
        dx = 100 * abs(plus_di - minus_di) / denom
        return float(dx)

    # ── ATR ───────────────────────────────────────────────────────────────────

    def _atr(self, period: int = 20) -> float:
        n = len(self._closes)
        if n < 2:
            return 0.0

        highs  = np.array(self._highs,  dtype=float)
        lows   = np.array(self._lows,   dtype=float)
        closes = np.array(self._closes, dtype=float)
        k = min(period, n - 1)

        tr = np.maximum(highs[-k:] - lows[-k:],
             np.maximum(np.abs(highs[-k:] - closes[-k-1:-1]),
                        np.abs(lows[-k:]  - closes[-k-1:-1])))
        return float(tr.mean())

    # ── Ratio de volumen ──────────────────────────────────────────────────────

    def _vol_ratio(self, period: int = 20) -> float:
        vols = list(self._volumes)
        if len(vols) < 2:
            return 1.0
        avg = float(np.mean(vols[-period-1:-1])) if len(vols) > period else float(np.mean(vols[:-1]))
        return float(vols[-1] / avg) if avg > 0 else 1.0

    # ── Exponente de Hurst ────────────────────────────────────────────────────

    def _hurst(self, series: np.ndarray) -> float:
        """R/S Analysis para estimar exponente de Hurst.

        H ≈ 0.5 → random walk
        H > 0.5 → tendencia (momentum)
        H < 0.5 → reversión a media
        """
        n = len(series)
        if n < 20:
            return 0.5
        try:
            lags = [2, 4, 8, 16, 32, 64]
            lags = [l for l in lags if l < n // 2]
            if len(lags) < 3:
                return 0.5

            rs_vals = []
            for lag in lags:
                rs_list = []
                for start in range(0, n - lag, lag):
                    sub = series[start:start + lag]
                    mean_s = sub.mean()
                    dev = np.cumsum(sub - mean_s)
                    r = dev.max() - dev.min()
                    s = sub.std(ddof=1)
                    if s > 1e-10:
                        rs_list.append(r / s)
                if rs_list:
                    rs_vals.append(np.mean(rs_list))

            if len(rs_vals) < 3:
                return 0.5

            log_lags = np.log(lags[:len(rs_vals)])
            log_rs   = np.log(rs_vals)
            hurst    = float(np.polyfit(log_lags, log_rs, 1)[0])
            return float(np.clip(hurst, 0.0, 1.0))
        except Exception:
            return 0.5


# ── Bar + BarAggregator ───────────────────────────────────────────────────────

@dataclass
class Bar:
    """Vela OHLCV con metadatos de tiempo."""
    open:      float
    high:      float
    low:       float
    close:     float
    volume:    float
    ts_open:   float           # UNIX timestamp apertura (boundary del intervalo)
    ts_close:  float = 0.0    # UNIX timestamp cierre real (0 si aún abierta)
    is_closed: bool  = False


class BarAggregator:
    """Agrega ticks de precio en velas OHLCV de N segundos (default 60 = 1m).

    Usa boundaries UNIX (floor al intervalo) para alineación exacta:
    tick en t=1234 con bar_seconds=60 → ts_open=1200.

    Uso::

        agg = BarAggregator(bar_seconds=60)
        for tick in ticks:
            closed = agg.update(close=tick.price, high=tick.high,
                                low=tick.low, volume=tick.vol)
            if closed:
                print(f"Vela cerrada: {closed.open} → {closed.close}")

    Propiedades:
        last_closed_bar  — vela más reciente cerrada
        prev_closed_bar  — penúltima vela cerrada
        current_bar      — vela en formación (aún no cerrada)
        seconds_to_close — segundos hasta que cierra la vela actual
        avg_volume(n)    — volumen promedio de las últimas n velas
    """

    def __init__(self, bar_seconds: int = 60) -> None:
        self._bar_seconds = bar_seconds
        self._current: Optional[Bar] = None
        self._closed:  deque[Bar]    = deque(maxlen=200)
        self._vol_history: deque[float] = deque(maxlen=200)

    def update(
        self,
        close:  float,
        high:   float,
        low:    float,
        volume: float,
        ts: float | None = None,
    ) -> Optional[Bar]:
        """Procesa tick; retorna la vela recién cerrada, o None si la vela sigue abierta."""
        now    = ts if ts is not None else time.time()
        bar_ts = float(int(now // self._bar_seconds) * self._bar_seconds)

        if self._current is None:
            self._current = Bar(
                open=close, high=high, low=low, close=close,
                volume=volume, ts_open=bar_ts,
            )
            return None

        if bar_ts == self._current.ts_open:
            # Mismo intervalo: actualizar OHLCV
            self._current.high   = max(self._current.high, high)
            self._current.low    = min(self._current.low,  low)
            self._current.close  = close
            self._current.volume += volume
            return None

        # Nuevo intervalo → cerrar barra actual
        closed           = self._current
        closed.ts_close  = now
        closed.is_closed = True
        self._closed.append(closed)
        self._vol_history.append(closed.volume)

        # Abrir nueva barra
        self._current = Bar(
            open=close, high=high, low=low, close=close,
            volume=volume, ts_open=bar_ts,
        )
        return closed

    @property
    def last_closed_bar(self) -> Optional[Bar]:
        """Última vela cerrada (la más reciente)."""
        return self._closed[-1] if self._closed else None

    @property
    def prev_closed_bar(self) -> Optional[Bar]:
        """Penúltima vela cerrada."""
        return self._closed[-2] if len(self._closed) >= 2 else None

    @property
    def current_bar(self) -> Optional[Bar]:
        """Vela actualmente en formación (aún no cerrada)."""
        return self._current

    def seconds_to_close(self, ts: float | None = None) -> float:
        """Segundos que faltan para cerrar la vela actual."""
        if self._current is None:
            return float(self._bar_seconds)
        now     = ts if ts is not None else time.time()
        bar_end = self._current.ts_open + self._bar_seconds
        return max(0.0, bar_end - now)

    def avg_volume(self, n_bars: int = 20) -> float:
        """Volumen promedio de las últimas n velas cerradas."""
        if not self._vol_history:
            return 0.0
        recent = list(self._vol_history)[-n_bars:]
        return sum(recent) / len(recent)

    def n_closed(self) -> int:
        """Número de velas cerradas en el buffer."""
        return len(self._closed)
