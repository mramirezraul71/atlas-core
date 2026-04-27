"""
backtest/lean_simulator.py — ATLAS-Quant internal GBM simulator (compat shim path)

DEPRECATION (F1): este módulo mantiene la implementación histórica por
compatibilidad de imports. El nombre canónico nuevo es
`atlas_code_quant.backtest.internal_gbm_simulator`.

Motor de backtesting interno. Genera datos OHLCV sintéticos realistas y
replica el pipeline completo de LiveLoop para producir datasets de entrenamiento
sin dependencias externas (no TA-lib, no yfinance, no Tradier).

IMPORTANTE: este módulo NO implementa QuantConnect LEAN.

Arquitectura:
    LeanSimulator
    ├── generate_historical_data()   GBM + régimen Markov → OHLCV 5min
    ├── run_atlas_strategy()         Replica LiveLoop → trades_df + features_df
    └── generate_training_dataset()  Multi-símbolo → CSV/SQLite 1M+ filas

Indicadores (pure numpy):
    RSI-14, MACD(12,26,9), Volume ratio, ATR-14,
    BB-pct-20, OBV-norm, CVD, Hurst-simple, ADX-14

Score pipeline (producción):
    motif_edge  (trend alignment + autocorrelation proxy)
    tin_score   (logistic de features normalizadas)
    mtf_coherence (alineación 1m/5m/15m)
    regime_conf (ADX + retorno 20d)
    score = 0.30×motif + 0.30×tin + 0.20×mtf + 0.20×regime

Uso::

    sim = LeanSimulator()
    ohlcv = sim.generate_historical_data("SPY", years=5)
    trades, features = sim.run_atlas_strategy("SPY", ohlcv)
    paths = sim.generate_training_dataset(["SPY","QQQ","IWM"], years=5)
    print(f"Trades: {paths['trades_csv']}, Features: {paths['features_csv']}")
"""
from __future__ import annotations

import csv
import logging
import math
import os
import sqlite3
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

# ── Scanner universe imports ──────────────────────────────────────────────────
try:
    from atlas_code_quant.scanner.universe_catalog import ScannerUniverseCatalog
    from atlas_code_quant.scanner.asset_classifier import (
        AssetClass,
        AssetProfile,
        classify_asset,
    )
    from atlas_code_quant.scanner.etf_universe import get_all_etf_symbols, get_liquid_etfs
    from atlas_code_quant.scanner.index_universe import get_all_index_symbols
    from atlas_code_quant.scanner.crypto_universe import get_all_crypto_symbols
    from atlas_code_quant.scanner.futures_universe import get_all_futures_symbols
    _SCANNER_AVAILABLE = True
except ImportError:
    _SCANNER_AVAILABLE = False

logger = logging.getLogger("atlas.backtest.lean")


# ── Parámetros por símbolo ─────────────────────────────────────────────────────

_SYMBOL_PARAMS: dict[str, dict] = {
    "SPY":  {"S0": 450.0, "mu_ann": 0.12, "sigma_ann": 0.14, "avg_vol": 80_000_000},
    "QQQ":  {"S0": 380.0, "mu_ann": 0.16, "sigma_ann": 0.18, "avg_vol": 50_000_000},
    "IWM":  {"S0": 185.0, "mu_ann": 0.09, "sigma_ann": 0.20, "avg_vol": 30_000_000},
    "AAPL": {"S0": 180.0, "mu_ann": 0.22, "sigma_ann": 0.24, "avg_vol": 60_000_000},
    "NVDA": {"S0": 600.0, "mu_ann": 0.45, "sigma_ann": 0.50, "avg_vol": 40_000_000},
    "TSLA": {"S0": 250.0, "mu_ann": 0.35, "sigma_ann": 0.55, "avg_vol": 90_000_000},
    "MSFT": {"S0": 380.0, "mu_ann": 0.20, "sigma_ann": 0.20, "avg_vol": 25_000_000},
    "AMZN": {"S0": 185.0, "mu_ann": 0.18, "sigma_ann": 0.26, "avg_vol": 35_000_000},
}

# Régimen: multiplicadores de drift y sigma
_REGIME_PARAMS: dict[str, dict] = {
    "BULL":     {"mu_mult": 1.8,  "sigma_mult": 0.75},
    "SIDEWAYS": {"mu_mult": 0.2,  "sigma_mult": 0.55},
    "BEAR":     {"mu_mult": -1.5, "sigma_mult": 1.40},
}

# Matriz de transición diaria (BULL→B/S/Be, SIDEWAYS→B/S/Be, BEAR→B/S/Be)
_TRANSITION: np.ndarray = np.array([
    [0.92, 0.06, 0.02],   # desde BULL
    [0.05, 0.88, 0.07],   # desde SIDEWAYS
    [0.03, 0.08, 0.89],   # desde BEAR
])
_REGIME_STATES = ["BULL", "SIDEWAYS", "BEAR"]

# Trading calendar: 252 días/año, 78 barras de 5min/día (9:30-16:00 ET)
_TRADING_DAYS   = 252
_BARS_PER_DAY   = 78
_DT_5MIN        = 1.0 / (_TRADING_DAYS * _BARS_PER_DAY)  # fracción anual

# Comisiones y slippage (modelo simplificado)
_SLIPPAGE_BPS   = 2.0    # 2 bps por lado
_COMMISSION_PER_SHARE = 0.005  # $0.005/share (IB-like)
_TRADIER_COMMISSION = 0.35   # $0.35/order (Tradier flat)

# ── Params GBM por clase de activo (fallback cuando símbolo no está en _SYMBOL_PARAMS) ─
# Estructura: {asset_class_value: {S0, mu_ann, sigma_ann, avg_vol}}
_ASSET_CLASS_PARAMS: Dict[str, Dict] = {
    "equity_stock":  {"S0": 80.0,    "mu_ann": 0.14, "sigma_ann": 0.28, "avg_vol": 5_000_000},
    "equity_etf":    {"S0": 200.0,   "mu_ann": 0.11, "sigma_ann": 0.16, "avg_vol": 20_000_000},
    "index_option":  {"S0": 4500.0,  "mu_ann": 0.12, "sigma_ann": 0.14, "avg_vol": 0},
    "crypto":        {"S0": 45_000.0,"mu_ann": 0.80, "sigma_ann": 0.90, "avg_vol": 500_000_000},
    "future":        {"S0": 4500.0,  "mu_ann": 0.10, "sigma_ann": 0.15, "avg_vol": 0},
    "forex":         {"S0": 1.10,    "mu_ann": 0.01, "sigma_ann": 0.06, "avg_vol": 0},
    "unknown":       {"S0": 50.0,    "mu_ann": 0.10, "sigma_ann": 0.30, "avg_vol": 1_000_000},
}


def _build_full_universe() -> List[str]:
    """Construye el universo completo combinando todos los módulos del scanner.

    Orden de prioridad:
    1. ETFs líquidos (opciones activas)
    2. Índices con opciones
    3. Crypto (si _SCANNER_AVAILABLE)
    4. Futuros micro (capital reducido)

    Exclye crypto y futuros del universo por defecto; se activan con flags de entorno.
    """
    if not _SCANNER_AVAILABLE:
        return list(_SYMBOL_PARAMS.keys())

    include_crypto  = os.getenv("ATLAS_SCANNER_INCLUDE_CRYPTO",  "false").lower() == "true"
    include_futures = os.getenv("ATLAS_SCANNER_INCLUDE_FUTURES", "false").lower() == "true"

    syms: List[str] = []

    # ETFs líquidos (OI ≥ 50k) — núcleo del universo
    try:
        syms += get_liquid_etfs(min_oi=50_000)
    except Exception:
        syms += list(_SYMBOL_PARAMS.keys())

    # Índices con opciones (SPX, NDX, RUT…)
    try:
        syms += [s for s in get_all_index_symbols() if s not in syms]
    except Exception:
        pass

    # Crypto (opcional)
    if include_crypto:
        try:
            syms += [s for s in get_all_crypto_symbols() if s not in syms]
        except Exception:
            pass

    # Futuros micro (opcional)
    if include_futures:
        try:
            from atlas_code_quant.scanner.futures_universe import get_micro_futures
            syms += [s for s in get_micro_futures() if s not in syms]
        except Exception:
            pass

    # Dedup preservando orden
    seen: set = set()
    result = []
    for s in syms:
        if s not in seen:
            seen.add(s)
            result.append(s)

    logger.info("_build_full_universe: %d símbolos", len(result))
    return result

# Score pipeline — pesos de producción (sensitivity_analysis.py)
_W_MOTIF  = 0.30
_W_TIN    = 0.30
_W_MTF    = 0.20
_W_REGIME = 0.20

# Tiers
_TIER_FULL   = 0.75
_TIER_NORMAL = 0.65
_TIER_SMALL  = 0.55


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class SimConfig:
    """Configuración del simulador."""
    # GBM
    random_seed:     int  = 42
    gap_probability: float = 0.015   # P(gap day) = 1.5%
    gap_magnitude:   float = 0.015   # ±1.5% en gaps

    # Strategy
    hold_bars:       int  = 12       # ~60 min de hold por defecto
    sl_atr_mult:     float = 2.5
    tp_atr_mult:     float = 2.0
    min_bars_warmup: int  = 60       # barras de warm-up antes de operar

    # Kelly base fraction
    kelly_base:      float = 0.25
    capital:         float = 100_000.0

    # Output
    out_dir:         str  = "data/backtest"
    chunk_size:      int  = 50_000   # filas por chunk al escribir CSV


@dataclass
class TradeRecord:
    """Registro de un trade simulado."""
    timestamp:    str
    symbol:       str
    side:         str      # "long" / "short"
    qty:          int
    entry_price:  float
    exit_price:   float
    pnl:          float
    pnl_pct:      float
    signal_score: float
    motif_edge:   float
    tin_score:    float
    mtf_coh:      float
    regime_conf:  float
    tier:         str
    hold_bars:    int
    exit_reason:  str      # "tp" / "sl" / "time"


# ── Indicadores (pure numpy) ──────────────────────────────────────────────────

def _ema(arr: np.ndarray, span: int) -> np.ndarray:
    """EMA vectorizada sobre array 1D."""
    alpha = 2.0 / (span + 1)
    out   = np.empty_like(arr)
    out[0] = arr[0]
    for i in range(1, len(arr)):
        out[i] = alpha * arr[i] + (1 - alpha) * out[i - 1]
    return out


def _rsi(closes: np.ndarray, period: int = 14) -> np.ndarray:
    n   = len(closes)
    out = np.full(n, 50.0)
    if n < period + 1:
        return out
    delta = np.diff(closes, prepend=closes[0])
    gain  = np.where(delta > 0, delta, 0.0)
    loss  = np.where(delta < 0, -delta, 0.0)
    avg_g = _ema(gain, period)
    avg_l = _ema(loss, period)
    rs    = avg_g / (avg_l + 1e-10)
    out   = 100.0 - (100.0 / (1.0 + rs))
    return out


def _macd(closes: np.ndarray,
          fast: int = 12, slow: int = 26, signal: int = 9
          ) -> tuple[np.ndarray, np.ndarray]:
    """Devuelve (macd_line, histogram)."""
    ema_f  = _ema(closes, fast)
    ema_s  = _ema(closes, slow)
    line   = ema_f - ema_s
    sig    = _ema(line, signal)
    hist   = line - sig
    # Normalizar por precio
    norm   = hist / (np.abs(closes) + 1e-10) * 100
    return line, norm


def _atr(high: np.ndarray, low: np.ndarray, close: np.ndarray,
         period: int = 14) -> np.ndarray:
    prev_c = np.roll(close, 1)
    prev_c[0] = close[0]
    tr = np.maximum(high - low,
         np.maximum(np.abs(high - prev_c),
                    np.abs(low  - prev_c)))
    return _ema(tr, period)


def _bollinger_pct(closes: np.ndarray, window: int = 20) -> np.ndarray:
    """Posición del precio dentro de las bandas de Bollinger [0-1]."""
    n   = len(closes)
    out = np.full(n, 0.5)
    for i in range(window, n):
        w  = closes[i - window:i]
        mu = w.mean()
        sd = w.std(ddof=1) + 1e-10
        out[i] = (closes[i] - (mu - 2 * sd)) / (4 * sd)
    return np.clip(out, 0.0, 1.0)


def _obv_norm(closes: np.ndarray, volumes: np.ndarray,
              window: int = 50) -> np.ndarray:
    """OBV normalizado por z-score rolling."""
    signs = np.sign(np.diff(closes, prepend=closes[0]))
    obv   = np.cumsum(signs * volumes)
    n     = len(obv)
    out   = np.zeros(n)
    for i in range(window, n):
        w   = obv[i - window:i]
        mu  = w.mean()
        sd  = w.std(ddof=1) + 1e-10
        out[i] = (obv[i] - mu) / sd
    return np.clip(out, -3.0, 3.0)


def _volume_ratio(volumes: np.ndarray, window: int = 20) -> np.ndarray:
    """Ratio volumen actual / media rolling."""
    out = np.ones(len(volumes))
    for i in range(window, len(volumes)):
        avg = volumes[i - window:i].mean()
        out[i] = volumes[i] / (avg + 1e-10)
    return out


def _adx_simple(high: np.ndarray, low: np.ndarray, close: np.ndarray,
                period: int = 14) -> np.ndarray:
    """ADX simplificado (EMA de |DI+ - DI-| / (DI+ + DI-))."""
    n     = len(close)
    out   = np.full(n, 20.0)
    atr_v = _atr(high, low, close, period)
    dmp   = np.maximum(np.diff(high,  prepend=high[0]),  0.0)
    dmm   = np.maximum(np.diff(low[::-1], prepend=low[-1])[::-1] * -1, 0.0)
    for i in range(period, n):
        atr_i = atr_v[i]
        if atr_i < 1e-10:
            continue
        pdi  = dmp[i - period:i].mean() / atr_i * 100
        mdi  = dmm[i - period:i].mean() / atr_i * 100
        denom = pdi + mdi
        if denom > 1e-10:
            out[i] = abs(pdi - mdi) / denom * 100
    return out


def _hurst(closes: np.ndarray, window: int = 60) -> np.ndarray:
    """Hurst exponent approx rolling."""
    n   = len(closes)
    out = np.full(n, 0.5)
    lags = [4, 8, 16]
    for i in range(window, n):
        w = closes[i - window:i]
        try:
            rs_vals = []
            for lag in lags:
                sub = w[-lag:]
                dev = np.cumsum(sub - sub.mean())
                r   = dev.max() - dev.min()
                s   = sub.std(ddof=1) + 1e-10
                rs_vals.append(r / s)
            slope = float(np.polyfit(np.log(lags), np.log(rs_vals), 1)[0])
            out[i] = float(np.clip(slope, 0.0, 1.0))
        except Exception:
            pass
    return out


def _cvd(opens: np.ndarray, closes: np.ndarray,
         volumes: np.ndarray, window: int = 50) -> np.ndarray:
    """CVD proxy: OBV ponderado por (close-open)/range."""
    n     = len(closes)
    rng   = np.abs(closes - opens) + 1e-10
    delta = np.sign(closes - opens) * volumes * np.abs(closes - opens) / rng
    cum   = np.cumsum(delta)
    out   = np.zeros(n)
    for i in range(window, n):
        w   = cum[i - window:i]
        mu  = w.mean()
        sd  = w.std(ddof=1) + 1e-10
        out[i] = (cum[i] - mu) / sd
    return np.clip(out, -3.0, 3.0)


# ── Simulador principal ───────────────────────────────────────────────────────

class LeanSimulator:
    """Motor de backtesting LEAN para ATLAS-Quant.

    Genera datos OHLCV 5-min sintéticos con régimen Markov y replica
    el pipeline completo de LiveLoop para producir datasets de entrenamiento.

    Uso típico::

        # Universo completo del scanner (ETFs + índices + crypto opcional)
        sim = LeanSimulator(use_full_universe=True)
        paths = sim.generate_training_dataset(years=5)
        # → logs/lean_sim/trades_20260324.csv
        # → logs/lean_sim/features_20260324.csv
        # → logs/lean_sim/lean_sim_20260324.db  (SQLite)

        # Universo reducido para pruebas rápidas
        sim = LeanSimulator(use_full_universe=False)

        # Universo custom
        sim = LeanSimulator(symbols=["SPY","QQQ","IWM"])
    """

    def __init__(
        self,
        symbols: List[str] | None = None,
        config: SimConfig | None = None,
        use_full_universe: bool = False,
        out_dir: str | None = None,
    ) -> None:
        self.cfg        = config or SimConfig()
        if out_dir:
            self.cfg.out_dir = out_dir

        if symbols is not None:
            # Explicit list overrides everything
            self.symbols = list(symbols)
        elif use_full_universe:
            self.symbols = _build_full_universe()
        else:
            self.symbols = ["SPY", "QQQ", "IWM", "AAPL", "NVDA"]

        self.data_cache: Dict[str, pd.DataFrame] = {}
        self._rng       = np.random.default_rng(self.cfg.random_seed)
        print(f"LeanSimulator: {len(self.symbols)} símbolos | universe={'full' if use_full_universe else 'custom' if symbols else 'default'}")

    def _get_symbol_params(self, symbol: str) -> Dict:
        """Devuelve params GBM para un símbolo.

        Prioridad:
        1. _SYMBOL_PARAMS (tabla curada de conocidos)
        2. classify_asset() → _ASSET_CLASS_PARAMS[asset_class]
        3. Fallback "unknown"
        """
        if symbol in _SYMBOL_PARAMS:
            return _SYMBOL_PARAMS[symbol]

        if _SCANNER_AVAILABLE:
            try:
                profile = classify_asset(symbol)
                class_key = profile.asset_class.value  # e.g. "equity_etf"
                base = dict(_ASSET_CLASS_PARAMS.get(class_key, _ASSET_CLASS_PARAMS["unknown"]))
                # Ajustar volatilidad para leveraged ETFs (TQQQ/SQQQ/SPXL/SPXU)
                if class_key == "equity_etf" and any(
                    symbol.startswith(pfx) for pfx in ("TQQQ", "SQQQ", "SPXL", "SPXU", "UVXY", "SVXY")
                ):
                    base["sigma_ann"] *= 2.5
                    base["mu_ann"]    *= 2.0
                return base
            except Exception:
                pass

        return dict(_ASSET_CLASS_PARAMS["unknown"])

    # ── 1. Generación de datos históricos ─────────────────────────────────────

    def generate_historical_data(
        self,
        symbol: str,
        years: int = 5,
        start_price: float | None = None,
    ) -> pd.DataFrame:
        """Genera OHLCV 5-min sintético con GBM + régimen Markov.

        Args:
            symbol:      Ticker — se busca en _SYMBOL_PARAMS, si no existe usa SPY params.
            years:       Años de historia a generar.
            start_price: Precio inicial (None → usa S0 del símbolo).

        Returns:
            DataFrame con columnas [timestamp, open, high, low, close, volume, regime].
        """
        params  = self._get_symbol_params(symbol)
        S0      = start_price or params["S0"]
        mu_ann  = params["mu_ann"]
        sig_ann = params["sigma_ann"]
        avg_vol = params["avg_vol"]

        n_days   = int(years * _TRADING_DAYS)
        n_bars   = n_days * _BARS_PER_DAY

        logger.info("LeanSim: generando %s | %d años | %d barras", symbol, years, n_bars)
        t_start = time.perf_counter()

        # ── 1.1 Régimen diario (Markov) ────────────────────────────────────────
        regimes = self._generate_regime_sequence(n_days)

        # ── 1.2 GBM intraday 5-min ────────────────────────────────────────────
        opens  = np.empty(n_bars)
        highs  = np.empty(n_bars)
        lows   = np.empty(n_bars)
        closes = np.empty(n_bars)
        vols   = np.empty(n_bars)
        reg_labels = []

        price = S0

        # Intraday volatility pattern: U-shape (opening + closing higher vol)
        intraday_vol_mult = self._intraday_vol_profile()

        for d in range(n_days):
            regime     = regimes[d]
            rp         = _REGIME_PARAMS[regime]
            mu_eff     = mu_ann * rp["mu_mult"]
            sig_eff    = sig_ann * rp["sigma_mult"]
            gap_return = 0.0

            # Gap de apertura ocasional
            if self._rng.random() < self.cfg.gap_probability:
                gap_return = self._rng.normal(0, self.cfg.gap_magnitude)

            day_open = price * (1 + gap_return)
            base_idx = d * _BARS_PER_DAY

            bar_closes = np.empty(_BARS_PER_DAY)
            bar_opens  = np.empty(_BARS_PER_DAY)

            # Primer bar del día
            bar_opens[0] = day_open
            for b in range(_BARS_PER_DAY):
                iv = intraday_vol_mult[b]
                z  = self._rng.standard_normal()
                ret = (mu_eff * _DT_5MIN
                       + sig_eff * iv * math.sqrt(_DT_5MIN) * z)
                if b == 0:
                    c = day_open * (1 + ret)
                    bar_opens[0] = day_open
                else:
                    bar_opens[b] = bar_closes[b - 1]
                    c = bar_opens[b] * (1 + ret)
                bar_closes[b] = max(c, 0.01)

            # High/Low: spread around body proportional to volatility
            spread = sig_eff * math.sqrt(_DT_5MIN) * intraday_vol_mult
            for b in range(_BARS_PER_DAY):
                idx = base_idx + b
                o = bar_opens[b]
                c = bar_closes[b]
                h = max(o, c) * (1 + abs(self._rng.normal(0, spread[b] * 0.6)))
                l = min(o, c) * (1 - abs(self._rng.normal(0, spread[b] * 0.6)))
                opens[idx]  = round(o, 4)
                highs[idx]  = round(max(h, o, c), 4)
                lows[idx]   = round(min(l, o, c), 4)
                closes[idx] = round(c, 4)

                # Volumen correlacionado con |ret| + intraday pattern
                price_ret   = abs(c / (o + 1e-10) - 1)
                vol_base    = avg_vol / _BARS_PER_DAY
                vol_intra   = intraday_vol_mult[b]
                vol_noise   = self._rng.lognormal(0, 0.3)
                vols[idx]   = max(100, int(
                    vol_base * vol_intra * (1 + price_ret * 20) * vol_noise
                ))
                reg_labels.append(regime)

            price = bar_closes[-1]  # carry forward to next day

        # ── 1.3 Timestamps (trading calendar sintético) ───────────────────────
        timestamps = self._build_timestamps(n_days)

        elapsed = (time.perf_counter() - t_start) * 1000
        logger.info("LeanSim: %s generado en %.0f ms (%d barras)", symbol, elapsed, n_bars)

        df = pd.DataFrame({
            "timestamp": timestamps,
            "open":      opens,
            "high":      highs,
            "low":       lows,
            "close":     closes,
            "volume":    vols.astype(np.int64),
            "regime":    reg_labels,
        })
        self.data_cache[symbol] = df
        return df

    def _generate_regime_sequence(self, n_days: int) -> list[str]:
        """Markov chain 3 estados: BULL / SIDEWAYS / BEAR."""
        states  = [0]   # empieza en BULL
        cur     = 0
        for _ in range(n_days - 1):
            probs = _TRANSITION[cur]
            cur   = int(self._rng.choice(3, p=probs))
            states.append(cur)
        return [_REGIME_STATES[s] for s in states]

    @staticmethod
    def _intraday_vol_profile() -> np.ndarray:
        """Perfil U-shape de volatilidad intraday (78 barras de 5min)."""
        x      = np.linspace(0, 1, _BARS_PER_DAY)
        # U-shape: alta apertura, baja mediodía, moderada cierre
        profile = 1.0 + 1.2 * np.exp(-8 * x) + 0.4 * np.exp(-8 * (1 - x))
        return profile / profile.mean()

    @staticmethod
    def _build_timestamps(n_days: int) -> list[str]:
        """Genera timestamps NY 9:30-16:00 ET."""
        ts = []
        base = pd.Timestamp("2021-01-04 09:30:00")  # primer día de trading
        day  = 0
        biz_days_added = 0
        while biz_days_added < n_days:
            current = base + pd.Timedelta(days=day)
            if current.weekday() < 5:   # Mon-Fri
                for b in range(_BARS_PER_DAY):
                    t = current + pd.Timedelta(minutes=5 * b)
                    ts.append(str(t))
                biz_days_added += 1
            day += 1
        return ts

    # ── 2. Estrategia ATLAS ───────────────────────────────────────────────────

    def run_atlas_strategy(
        self,
        symbol: str,
        ohlcv_df: pd.DataFrame | None = None,
    ) -> tuple[pd.DataFrame, pd.DataFrame]:
        """Replica el pipeline completo de LiveLoop sobre OHLCV 5-min.

        Returns:
            (trades_df, features_df) — ambos DataFrames con todas las columnas
            necesarias para entrenar PatternLab.
        """
        if ohlcv_df is None:
            if symbol in self.data_cache:
                ohlcv_df = self.data_cache[symbol]
            else:
                raise ValueError(f"No hay datos para {symbol}. Llama generate_historical_data() primero.")

        logger.info("LeanSim: run_atlas_strategy %s | %d barras", symbol, len(ohlcv_df))
        t0 = time.perf_counter()

        # ── 2.1 Calcular features ─────────────────────────────────────────────
        feat_df = self._compute_features(ohlcv_df)

        # ── 2.2 Calcular componentes del score ────────────────────────────────
        closes  = feat_df["close"].values
        n       = len(feat_df)

        motif_arr  = self._compute_motif_edge(feat_df)
        tin_arr    = self._compute_tin_score(feat_df)
        mtf_arr    = self._compute_mtf_coherence(closes)
        regime_arr, regime_name = self._compute_regime(feat_df)

        # Score compuesto
        def _norm(x):
            return np.clip(x, 0.0, 1.0)

        motif_n = _norm((motif_arr + 1.0) / 2.0)   # [-1,1] → [0,1]
        score_arr = (_W_MOTIF * motif_n
                   + _W_TIN   * _norm(tin_arr)
                   + _W_MTF   * _norm(mtf_arr)
                   + _W_REGIME * _norm(regime_arr))
        score_arr = np.clip(score_arr, 0.0, 1.0)

        # ── 2.3 Tiers y dirección ─────────────────────────────────────────────
        atr_arr = feat_df["atr_14"].values

        trade_records: list[dict] = []
        warmup = self.cfg.min_bars_warmup

        bar_i = warmup
        while bar_i < n - self.cfg.hold_bars - 1:
            score = float(score_arr[bar_i])
            tier  = self._get_tier(score)

            if tier == "SKIP":
                bar_i += 1
                continue

            # Dirección: usa trend (motif) + RSI
            rsi_v    = float(feat_df["rsi_14"].values[bar_i])
            motif_v  = float(motif_arr[bar_i])
            side     = "long" if (motif_v > 0 and rsi_v < 70) else (
                        "short" if (motif_v < 0 and rsi_v > 30) else None)
            if side is None:
                bar_i += 1
                continue

            # Kelly sizing
            kelly_f  = self._kelly_factor(score, tier)
            price_e  = float(ohlcv_df["close"].values[bar_i])
            atr_e    = float(atr_arr[bar_i])
            qty      = self._compute_shares(kelly_f, price_e, atr_e)

            if qty == 0:
                bar_i += 1
                continue

            # ── 2.4 Simular fill y gestión de trade ───────────────────────────
            entry_price = self._apply_slippage(price_e, side, "entry")
            sl_dist     = atr_e * self.cfg.sl_atr_mult
            tp_dist     = atr_e * self.cfg.tp_atr_mult
            sl_price    = (entry_price - sl_dist if side == "long"
                          else entry_price + sl_dist)
            tp_price    = (entry_price + tp_dist if side == "long"
                          else entry_price - tp_dist)

            exit_bar = bar_i + 1
            exit_price = None
            exit_reason = "time"

            max_exit = min(bar_i + self.cfg.hold_bars, n - 1)
            for eb in range(bar_i + 1, max_exit + 1):
                c_h = float(ohlcv_df["high"].values[eb])
                c_l = float(ohlcv_df["low"].values[eb])
                if side == "long":
                    if c_l <= sl_price:
                        exit_price  = sl_price
                        exit_reason = "sl"
                        exit_bar    = eb
                        break
                    if c_h >= tp_price:
                        exit_price  = tp_price
                        exit_reason = "tp"
                        exit_bar    = eb
                        break
                else:
                    if c_h >= sl_price:
                        exit_price  = sl_price
                        exit_reason = "sl"
                        exit_bar    = eb
                        break
                    if c_l <= tp_price:
                        exit_price  = tp_price
                        exit_reason = "tp"
                        exit_bar    = eb
                        break

            if exit_price is None:
                exit_price  = float(ohlcv_df["close"].values[max_exit])
                exit_bar    = max_exit

            exit_price = self._apply_slippage(exit_price, side, "exit")
            commission  = qty * _COMMISSION_PER_SHARE * 2

            if side == "long":
                raw_pnl = (exit_price - entry_price) * qty - commission
            else:
                raw_pnl = (entry_price - exit_price) * qty - commission

            pnl_pct = raw_pnl / (self.cfg.capital + 1e-10)

            trade_records.append({
                "timestamp":    str(ohlcv_df["timestamp"].values[bar_i]),
                "symbol":       symbol,
                "side":         side,
                "qty":          qty,
                "entry_price":  round(entry_price, 4),
                "exit_price":   round(exit_price, 4),
                "pnl":          round(raw_pnl, 4),
                "pnl_pct":      round(pnl_pct * 100, 6),
                "signal_score": round(score, 4),
                "motif_edge":   round(float(motif_arr[bar_i]), 4),
                "tin_score":    round(float(tin_arr[bar_i]), 4),
                "mtf_coh":      round(float(mtf_arr[bar_i]), 4),
                "regime_conf":  round(float(regime_arr[bar_i]), 4),
                "regime":       regime_name[bar_i],
                "tier":         tier,
                "hold_bars":    exit_bar - bar_i,
                "exit_reason":  exit_reason,
            })

            # Avanzar más allá del trade (no solapar)
            bar_i = exit_bar + 1

        # ── 2.5 Construir features_df (todas las barras ejecutables) ──────────
        feat_export = feat_df.copy()
        feat_export["symbol"]       = symbol
        feat_export["motif_edge"]   = motif_arr
        feat_export["tin_score"]    = tin_arr
        feat_export["mtf_coh"]      = mtf_arr
        feat_export["regime_conf"]  = regime_arr
        feat_export["regime"]       = regime_name
        feat_export["signal_score"] = score_arr

        # Label: retorno forward 12 barras (~60 min)
        fwd_ret = np.roll(feat_export["close"].values, -12) / (feat_export["close"].values + 1e-10) - 1
        fwd_ret[-12:] = 0.0
        feat_export["fwd_ret_12"]  = fwd_ret
        feat_export["label_bin"]   = (fwd_ret > 0).astype(int)

        trades_df = pd.DataFrame(trade_records)

        elapsed = (time.perf_counter() - t0) * 1000
        logger.info(
            "LeanSim: %s → %d trades | %d feature rows | %.0f ms",
            symbol, len(trades_df), len(feat_export), elapsed,
        )
        return trades_df, feat_export

    # ── 3. Cómputo de features ────────────────────────────────────────────────

    def _compute_features(self, ohlcv: pd.DataFrame) -> pd.DataFrame:
        """Calcula indicadores técnicos desde OHLCV. Pure numpy, sin TA-lib."""
        o  = ohlcv["open"].values.astype(np.float64)
        h  = ohlcv["high"].values.astype(np.float64)
        l  = ohlcv["low"].values.astype(np.float64)
        c  = ohlcv["close"].values.astype(np.float64)
        v  = ohlcv["volume"].values.astype(np.float64)

        rsi_v    = _rsi(c, 14)
        _, macd_h = _macd(c, 12, 26, 9)
        atr_v    = _atr(h, l, c, 14)
        atr_n    = atr_v / (c + 1e-10)
        bb_v     = _bollinger_pct(c, 20)
        obv_v    = _obv_norm(c, v, 50)
        vol_r    = _volume_ratio(v, 20)
        adx_v    = _adx_simple(h, l, c, 14)
        hurst_v  = _hurst(c, 60)
        cvd_v    = _cvd(o, c, v, 50)
        ret_20   = np.zeros(len(c))
        ret_20[20:] = (c[20:] - c[:-20]) / (c[:-20] + 1e-10)

        return pd.DataFrame({
            "timestamp":   ohlcv["timestamp"].values,
            "close":       c,
            "rsi_14":      rsi_v,
            "macd_hist":   macd_h,
            "atr_14":      atr_v,
            "atr_norm":    atr_n,
            "bb_pct":      bb_v,
            "obv_norm":    obv_v,
            "volume_ratio": vol_r,
            "adx_14":      adx_v,
            "hurst":       hurst_v,
            "cvd":         cvd_v,
            "ret_20":      ret_20,
        })

    # ── 4. Componentes del score ───────────────────────────────────────────────

    def _compute_motif_edge(self, feat_df: pd.DataFrame) -> np.ndarray:
        """Motif edge proxy: trend alignment + autocorrelación de retornos.

        Combina:
          - Trend proxy: EMA21/EMA50 crossover normalizado
          - Autocorr proxy: correlación rolling 20 barras consigo misma lagged
          Resultado en [-1, 1] (0=neutral)
        """
        c   = feat_df["close"].values
        n   = len(c)
        out = np.zeros(n)

        ema21 = _ema(c, 21)
        ema50 = _ema(c, 50)

        # Trend: distancia relativa EMA21 vs EMA50 normalizada
        trend = (ema21 - ema50) / (ema50 + 1e-10) * 50   # amplificar
        trend = np.clip(trend, -1.0, 1.0)

        # Autocorr proxy: RSI centrado + MACD sign
        rsi_c  = (feat_df["rsi_14"].values - 50.0) / 50.0   # [-1,1]
        macd_s = np.sign(feat_df["macd_hist"].values)

        # Ponderado
        out = 0.50 * trend + 0.30 * rsi_c + 0.20 * macd_s
        return np.clip(out, -1.0, 1.0)

    def _compute_tin_score(self, feat_df: pd.DataFrame) -> np.ndarray:
        """TIN score proxy: función logística sobre features normalizadas.

        Aproxima el output de la TechnicalIndicatorNet sin requerir
        un modelo entrenado. Capta patrones de momentum + volatilidad.
        """
        rsi    = feat_df["rsi_14"].values
        macd   = feat_df["macd_hist"].values
        vol_r  = feat_df["volume_ratio"].values
        bb     = feat_df["bb_pct"].values
        obv    = feat_df["obv_norm"].values
        cvd    = feat_df["cvd"].values

        # Normalizar cada componente [0,1]
        rsi_n    = rsi / 100.0
        macd_n   = np.clip((macd + 0.5) / 1.0, 0.0, 1.0)
        vol_n    = np.clip(vol_r / 3.0, 0.0, 1.0)
        obv_n    = np.clip((obv + 3.0) / 6.0, 0.0, 1.0)
        cvd_n    = np.clip((cvd + 3.0) / 6.0, 0.0, 1.0)

        # Combinación lineal → logística
        z = (0.30 * rsi_n + 0.25 * macd_n + 0.15 * vol_n
           + 0.15 * bb    + 0.10 * obv_n  + 0.05 * cvd_n)

        # Logística centrada en 0.5
        logit = (z - 0.5) * 6.0
        return 1.0 / (1.0 + np.exp(-logit))

    def _compute_mtf_coherence(self, closes: np.ndarray) -> np.ndarray:
        """MTF coherence: alineación de tendencia en 1m/5m/15m simulados.

        Submuestrea closes a distintas resoluciones y calcula si
        la dirección de tendencia coincide.
        """
        n   = len(closes)
        out = np.full(n, 0.5)

        ema_steps = [1, 5, 15]   # "timeframes" relativos

        for i in range(60, n):
            directions = []
            for step in ema_steps:
                # Submuestrear
                if step > 1:
                    sub = closes[max(0, i - 60 * step):i:step]
                else:
                    sub = closes[max(0, i - 60):i]
                # Usar trend simple: último valor vs media
                if len(sub) < 5:
                    directions.append(0)
                    continue
                ema_f = sub[-5:].mean()
                ema_s = sub.mean()
                directions.append(1 if ema_f > ema_s else -1)
            if directions:
                n_agree = sum(1 for d in directions if d == directions[0] and d != 0)
                out[i] = n_agree / len(directions)

        return np.clip(out, 0.0, 1.0)

    def _compute_regime(
        self, feat_df: pd.DataFrame
    ) -> tuple[np.ndarray, list[str]]:
        """Clasifica régimen desde features: Bull/Sideways/Bear.

        Returns:
            (confidence_array [0-1], regime_name_list)
        """
        adx    = feat_df["adx_14"].values
        ret_20 = feat_df["ret_20"].values
        hurst  = feat_df["hurst"].values
        rsi    = feat_df["rsi_14"].values

        n      = len(adx)
        conf   = np.full(n, 0.5)
        names  = ["SIDEWAYS"] * n

        for i in range(n):
            a = adx[i]
            r = ret_20[i]
            h = hurst[i]

            trending = a > 20
            if trending and r > 0.015 and rsi[i] > 50:
                regime = "BULL"
                c = min(0.95, 0.60 + a / 100 + abs(r) * 5)
            elif trending and r < -0.015 and rsi[i] < 50:
                regime = "BEAR"
                c = min(0.95, 0.60 + a / 100 + abs(r) * 5)
            else:
                regime = "SIDEWAYS"
                c = min(0.80, 0.50 + (1.0 - h) * 0.3)

            conf[i]  = c
            names[i] = regime

        return conf, names

    # ── 5. Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _get_tier(score: float) -> str:
        if score >= _TIER_FULL:   return "FULL"
        if score >= _TIER_NORMAL: return "NORMAL"
        if score >= _TIER_SMALL:  return "SMALL"
        return "SKIP"

    @staticmethod
    def _kelly_factor(score: float, tier: str) -> float:
        if tier == "SKIP":   return 0.0
        if tier == "SMALL":  return 0.50
        return min(2.0, max(0.5, score * 3.0))

    def _compute_shares(self, kelly_f: float, price: float, atr: float) -> int:
        if price <= 0 or atr <= 0 or kelly_f == 0:
            return 0
        sl_dist      = atr * self.cfg.sl_atr_mult
        max_risk_usd = self.cfg.capital * 0.01   # 1% capital
        shares_risk  = int(max_risk_usd / sl_dist) if sl_dist > 0 else 0
        kelly_cap    = self.cfg.capital * self.cfg.kelly_base * kelly_f
        shares_kelly = int(kelly_cap / price)
        return max(0, min(shares_risk, shares_kelly))

    @staticmethod
    def _apply_slippage(price: float, side: str, direction: str) -> float:
        slip = price * _SLIPPAGE_BPS / 10_000
        if direction == "entry":
            return price + slip if side == "long" else price - slip
        else:
            return price - slip if side == "long" else price + slip

    # ── 6. Generación masiva de dataset ───────────────────────────────────────

    def generate_training_dataset(
        self,
        symbols: list[str] | None = None,
        years: int = 5,
        out_dir: str | None = None,
    ) -> dict[str, str]:
        """Genera dataset de entrenamiento multi-símbolo.

        Para cada símbolo:
          1. Genera OHLCV 5-min (GBM + régimen)
          2. Ejecuta estrategia Atlas
          3. Acumula trades + features

        Exporta:
          trades_YYYYMMDD.csv   — trades ejecutados (entrada/salida/PnL)
          features_YYYYMMDD.csv — todas las barras con features + label

        Target: 1M+ filas en features_csv con 5+ símbolos × 5 años.

        Returns:
            dict con paths: {"trades_csv": ..., "features_csv": ..., "n_trades": ..., "n_features": ...}
        """
        syms    = symbols or self.symbols
        out_dir = Path(out_dir or self.cfg.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        date_tag       = time.strftime("%Y%m%d")
        trades_path    = out_dir / f"trades_{date_tag}.csv"
        features_path  = out_dir / f"features_{date_tag}.csv"
        db_path        = out_dir / f"lean_sim_{date_tag}.db"

        n_trades_total   = 0
        n_features_total = 0
        trades_header_written   = False
        features_header_written = False

        # Per-symbol stats para retornar métricas agregadas
        symbol_stats: Dict[str, Dict] = {}
        all_trades_for_db: List[pd.DataFrame] = []

        logger.info(
            "LeanSim: generate_training_dataset | syms=%d years=%d target=1M+",
            len(syms), years,
        )
        t_global = time.perf_counter()

        for sym in syms:
            logger.info("LeanSim: procesando %s ...", sym)
            # Generar datos
            try:
                ohlcv = self.generate_historical_data(sym, years=years)
            except Exception as e:
                logger.error("LeanSim: error generando %s: %s", sym, e)
                continue

            # Ejecutar estrategia
            try:
                trades_df, feat_df = self.run_atlas_strategy(sym, ohlcv)
            except Exception as e:
                logger.error("LeanSim: error run_atlas_strategy %s: %s", sym, e)
                continue

            # Escribir trades en chunks
            if not trades_df.empty:
                trades_df.to_csv(
                    trades_path,
                    mode   = "a",
                    index  = False,
                    header = not trades_header_written,
                )
                trades_header_written  = True
                n_trades_total        += len(trades_df)
                all_trades_for_db.append(trades_df)

                # Stats por símbolo
                symbol_stats[sym] = self.compute_metrics(trades_df)

            # Escribir features en chunks (sin columna regime ya en features)
            feat_export = feat_df.drop(columns=["regime"], errors="ignore")
            chunk       = self.cfg.chunk_size

            for start in range(0, len(feat_export), chunk):
                chunk_df = feat_export.iloc[start:start + chunk]
                chunk_df.to_csv(
                    features_path,
                    mode   = "a",
                    index  = False,
                    header = not features_header_written,
                )
                features_header_written  = True
            n_features_total += len(feat_export)

            logger.info(
                "LeanSim: %s → trades=%d features=%d (acum: T=%d F=%d)",
                sym, len(trades_df), len(feat_export),
                n_trades_total, n_features_total,
            )

        # ── Exportar SQLite ───────────────────────────────────────────────────
        db_exported = False
        if all_trades_for_db:
            try:
                all_trades = pd.concat(all_trades_for_db, ignore_index=True)
                self.export_to_sqlite(all_trades, str(db_path), symbol_stats)
                db_exported = True
            except Exception as e:
                logger.error("LeanSim: export_to_sqlite falló: %s", e)

        elapsed = (time.perf_counter() - t_global)

        # Sharpe promedio entre símbolos con datos
        sharpe_vals = [v.get("sharpe", 0) for v in symbol_stats.values() if isinstance(v, dict)]
        sharpe_avg  = round(sum(sharpe_vals) / len(sharpe_vals), 3) if sharpe_vals else 0.0

        logger.info(
            "LeanSim: COMPLETADO | trades=%d features=%d | sharpe_avg=%.3f | %.1fs",
            n_trades_total, n_features_total, sharpe_avg, elapsed,
        )
        print(
            f"  LeanSim completado: {n_trades_total:,} trades | "
            f"{n_features_total:,} feature rows | sharpe_avg={sharpe_avg} | {elapsed:.1f}s"
        )
        return {
            "trades_csv":        str(trades_path),
            "trades_path":       str(trades_path),    # alias para PatternLab hook
            "features_csv":      str(features_path),
            "features_path":     str(features_path),  # alias
            "db_path":           str(db_path) if db_exported else None,
            "n_trades":          n_trades_total,
            "total_trades":      n_trades_total,       # alias para PatternLab hook
            "n_features":        n_features_total,
            "sharpe_avg":        sharpe_avg,
            "trades_por_symbol": {s: v.get("n_trades", 0) for s, v in symbol_stats.items()},
            "elapsed_s":         round(elapsed, 2),
        }

    # ── 6b. Exportar SQLite ───────────────────────────────────────────────────

    def export_to_sqlite(
        self,
        trades_df: pd.DataFrame,
        db_path: str,
        symbol_stats: Dict[str, Dict] | None = None,
    ) -> None:
        """Exporta trades a SQLite compatible con TradingJournalService.

        Tablas:
          - trades         : filas de TradeRecord
          - symbol_metrics : métricas por símbolo (win_rate, sharpe, etc.)
          - sim_metadata   : metadatos de la simulación
        """
        db_path = str(db_path)
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)

        with sqlite3.connect(db_path) as conn:
            # ── Tabla trades ─────────────────────────────────────────────────
            trades_df.to_sql("trades", conn, if_exists="replace", index=False)

            # ── Tabla symbol_metrics ──────────────────────────────────────────
            if symbol_stats:
                rows = []
                for sym, m in symbol_stats.items():
                    if isinstance(m, dict):
                        rows.append({
                            "symbol":           sym,
                            "n_trades":         m.get("n_trades", 0),
                            "win_rate":         m.get("win_rate", 0),
                            "profit_factor":    m.get("profit_factor", 0),
                            "sharpe":           m.get("sharpe", 0),
                            "max_drawdown_pct": m.get("max_drawdown_pct", 0),
                            "total_pnl":        m.get("total_pnl", 0),
                            "final_equity":     m.get("final_equity", 0),
                            "avg_score":        m.get("avg_score", 0),
                        })
                if rows:
                    pd.DataFrame(rows).to_sql(
                        "symbol_metrics", conn, if_exists="replace", index=False
                    )

            # ── Tabla sim_metadata ────────────────────────────────────────────
            meta = pd.DataFrame([{
                "generated_at":  time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "n_symbols":     trades_df["symbol"].nunique() if "symbol" in trades_df else 0,
                "n_trades":      len(trades_df),
                "capital":       self.cfg.capital,
                "kelly_base":    self.cfg.kelly_base,
                "sl_atr_mult":   self.cfg.sl_atr_mult,
                "tp_atr_mult":   self.cfg.tp_atr_mult,
                "score_weights": f"motif={_W_MOTIF} tin={_W_TIN} mtf={_W_MTF} regime={_W_REGIME}",
                "simulator_ver": "1.1",
            }])
            meta.to_sql("sim_metadata", conn, if_exists="replace", index=False)

            conn.execute("CREATE INDEX IF NOT EXISTS idx_trades_symbol ON trades(symbol)")
            conn.execute("CREATE INDEX IF NOT EXISTS idx_trades_ts ON trades(timestamp)")
            conn.commit()

        logger.info("LeanSim: SQLite exportado → %s | %d trades", db_path, len(trades_df))

    # ── 7. Métricas de backtest ───────────────────────────────────────────────

    def compute_metrics(self, trades_df: pd.DataFrame) -> Dict:
        """Calcula métricas de backtest sobre un trades_df."""
        if trades_df.empty:
            return {"error": "sin trades"}

        pnls  = trades_df["pnl"].values
        n     = len(pnls)
        wins  = pnls > 0
        gross_profit = pnls[wins].sum()
        gross_loss   = abs(pnls[~wins].sum())

        equity = np.cumsum(pnls) + self.cfg.capital
        peak   = np.maximum.accumulate(equity)
        dd     = (peak - equity) / (peak + 1e-10)

        tier_dist = trades_df["tier"].value_counts().to_dict() if "tier" in trades_df else {}

        return {
            "n_trades":        n,
            "win_rate":        round(wins.mean() * 100, 2),
            "profit_factor":   round(gross_profit / (gross_loss + 1e-10), 3),
            "total_pnl":       round(pnls.sum(), 2),
            "avg_pnl":         round(pnls.mean(), 4),
            "avg_win":         round(pnls[wins].mean() if wins.any() else 0, 4),
            "avg_loss":        round(pnls[~wins].mean() if (~wins).any() else 0, 4),
            "max_drawdown_pct":round(dd.max() * 100, 2),
            "sharpe":          round(
                pnls.mean() / (pnls.std(ddof=1) + 1e-10) * math.sqrt(252 * _BARS_PER_DAY / 12), 3
            ),
            "final_equity":    round(float(equity[-1]), 2),
            "tier_dist":       tier_dist,
            "avg_score":       round(float(trades_df["signal_score"].mean()), 4),
        }
