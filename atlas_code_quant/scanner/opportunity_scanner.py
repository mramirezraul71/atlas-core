"""Atlas Code-Quant — Escáner permanente de oportunidades explicable."""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from math import isfinite
from typing import Any

import pandas as pd

from config.settings import settings
from data.feed import MarketFeed

logger = logging.getLogger("quant.scanner")

TIMEFRAME_ORDER = ["1d", "4h", "1h", "15m", "5m"]
HIGHER_TIMEFRAME_MAP = {
    "5m": "15m",
    "15m": "1h",
    "1h": "4h",
    "4h": "1d",
    "1d": None,
}
HORIZON_BARS = {"5m": 3, "15m": 4, "1h": 6, "4h": 4, "1d": 3}
METHOD_ORDER = ["trend_ema_stack", "breakout_donchian", "rsi_pullback_trend", "ml_directional"]
EVIDENCE_LIBRARY: dict[str, dict[str, Any]] = {
    "trend_ema_stack": {
        "label": "Tendencia estructural",
        "family": "seguimiento de tendencia",
        "evidence_score": 0.88,
        "description": "Precio y medias alineadas en favor de la tendencia principal.",
        "sources": [
            {
                "title": "Time Series Momentum",
                "url": "https://www.nber.org/papers/w18169.pdf",
                "note": "Documenta persistencia de momentum temporal en múltiples activos.",
            },
            {
                "title": "A Quantitative Approach to Tactical Asset Allocation",
                "url": "https://mebfaber.com/wp-content/uploads/2010/10/SSRN-id962461.pdf",
                "note": "Muestra valor de filtros de tendencia/medias en asignación táctica.",
            },
            {
                "title": "Trend-Following: What's Not to Like?",
                "url": "https://www.man.com/documents/download/01f5a-84d88-8d0a5-5a967/Man_AHL_Insights_Trend-Following_What%E2%80%99s_Not_to_Like_English_%28United_States%29_04-05-2023.pdf",
                "note": "Refuerza evidencia de carteras multi-modelo de tendencia.",
            },
        ],
    },
    "breakout_donchian": {
        "label": "Breakout con expansión",
        "family": "continuación",
        "evidence_score": 0.82,
        "description": "Ruptura de canal con ATR/volumen confirmando desplazamiento.",
        "sources": [
            {
                "title": "Time Series Momentum",
                "url": "https://www.nber.org/papers/w18169.pdf",
                "note": "Las rupturas son una implementación clásica del momentum temporal.",
            },
            {
                "title": "Trend-Following: What's Not to Like?",
                "url": "https://www.man.com/documents/download/01f5a-84d88-8d0a5-5a967/Man_AHL_Insights_Trend-Following_What%E2%80%99s_Not_to_Like_English_%28United_States%29_04-05-2023.pdf",
                "note": "La continuación de tendencia sigue siendo núcleo operativo en CTA modernos.",
            },
        ],
    },
    "rsi_pullback_trend": {
        "label": "Pullback dentro de tendencia",
        "family": "mean reversion condicionada",
        "evidence_score": 0.66,
        "description": "Busca retrocesos en tendencia dominante en vez de reversión aislada.",
        "sources": [
            {
                "title": "Reversal, Momentum, and Price Pressure",
                "url": "https://www.jstor.org/stable/2328885",
                "note": "La reversión existe, pero conviene acotarla a filtros de contexto.",
            },
            {
                "title": "Short-Term Reversal",
                "url": "https://www.nber.org/papers/w24748.pdf",
                "note": "La reversión de corto plazo existe, pero es inestable; se usa solo con filtro de tendencia.",
            },
        ],
    },
    "ml_directional": {
        "label": "Modelo direccional local",
        "family": "predictivo local",
        "evidence_score": 0.58,
        "description": "Modelo supervisado local; se considera apoyo y no fuente única de verdad.",
        "sources": [
            {
                "title": "Control interno ATLAS",
                "url": "",
                "note": "Se usa como refuerzo probabilístico, no como criterio autónomo único.",
            },
        ],
    },
    "relative_strength_overlay": {
        "label": "Momentum relativo",
        "family": "ranking de universo",
        "evidence_score": 0.84,
        "description": "Prioriza activos con mejor fuerza relativa frente al universo observado.",
        "sources": [
            {
                "title": "Returns to Buying Winners and Selling Losers",
                "url": "https://www.jstor.org/stable/2328883",
                "note": "Base robusta del momentum cross-sectional.",
            },
            {
                "title": "Breadth Momentum and Vigilant Asset Allocation",
                "url": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=3002624",
                "note": "Muestra valor de momentum absoluto/relativo y protección de drawdown.",
            },
        ],
    },
    "multi_timeframe_confirmation": {
        "label": "Confirmación de temporalidad superior",
        "family": "gobernanza ATLAS",
        "evidence_score": 0.72,
        "description": "Filtro operativo para reducir señales aisladas en marcos bajos.",
        "sources": [
            {
                "title": "Criterio de gobernanza ATLAS",
                "url": "",
                "note": "Se aplica como filtro de calidad, no como promesa de edge universal.",
            },
        ],
    },
}


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if isfinite(out) else default
    except (TypeError, ValueError):
        return default


def _pct_change(a: float, b: float) -> float:
    if not b:
        return 0.0
    return (a / b - 1.0) * 100.0


def _rsi(series: pd.Series, period: int = 14) -> pd.Series:
    delta = series.diff()
    gains = delta.clip(lower=0)
    losses = (-delta).clip(lower=0)
    avg_gain = gains.ewm(alpha=1 / period, adjust=False, min_periods=period).mean()
    avg_loss = losses.ewm(alpha=1 / period, adjust=False, min_periods=period).mean()
    rs = avg_gain / avg_loss.replace(0, pd.NA)
    return 100 - (100 / (1 + rs))


def _atr(df: pd.DataFrame, period: int = 14) -> pd.Series:
    high = df["high"]
    low = df["low"]
    close = df["close"]
    prev_close = close.shift(1)
    tr = pd.concat(
        [
            (high - low).abs(),
            (high - prev_close).abs(),
            (low - prev_close).abs(),
        ],
        axis=1,
    ).max(axis=1)
    return tr.rolling(period).mean()


def _direction_label(direction: int) -> str:
    if direction > 0:
        return "alcista"
    if direction < 0:
        return "bajista"
    return "neutral"


def _timeframes_sorted(values: list[str]) -> list[str]:
    order = {name: idx for idx, name in enumerate(TIMEFRAME_ORDER)}
    return sorted(values, key=lambda item: order.get(item, 99))


@dataclass(slots=True)
class ScannerConfig:
    enabled: bool
    source: str
    scan_interval_sec: int
    min_signal_strength: float
    min_local_win_rate_pct: float
    min_selection_score: float
    max_candidates: int
    require_higher_tf_confirmation: bool
    universe: list[str]
    timeframes: list[str]
    activity_limit: int
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "source": self.source,
            "scan_interval_sec": self.scan_interval_sec,
            "min_signal_strength": self.min_signal_strength,
            "min_local_win_rate_pct": self.min_local_win_rate_pct,
            "min_selection_score": self.min_selection_score,
            "max_candidates": self.max_candidates,
            "require_higher_tf_confirmation": self.require_higher_tf_confirmation,
            "universe": list(self.universe),
            "timeframes": list(self.timeframes),
            "activity_limit": self.activity_limit,
            "notes": self.notes,
        }


class OpportunityScannerService:
    """Escáner continuo con explicación explícita y criterios gobernados."""

    def __init__(self) -> None:
        self._config = ScannerConfig(
            enabled=settings.scanner_enabled,
            source=settings.scanner_source,
            scan_interval_sec=settings.scanner_scan_interval_sec,
            min_signal_strength=settings.scanner_min_signal_strength,
            min_local_win_rate_pct=settings.scanner_min_local_win_rate_pct,
            min_selection_score=settings.scanner_min_selection_score,
            max_candidates=settings.scanner_max_candidates,
            require_higher_tf_confirmation=settings.scanner_require_higher_tf_confirmation,
            universe=list(settings.scanner_universe),
            timeframes=_timeframes_sorted(list(settings.scanner_timeframes)),
            activity_limit=settings.scanner_activity_limit,
            notes="fase paper: escáner explicable y no ejecutor",
        )
        self._feed: MarketFeed | None = None
        self._ml_strategy = None
        self._task: asyncio.Task | None = None
        self._loop_lock = asyncio.Lock()
        self._state_lock = threading.RLock()
        self._running = False
        self._current_symbol: str | None = None
        self._current_timeframe: str | None = None
        self._current_step: str = "inactivo"
        self._cycle_count = 0
        self._last_cycle_at: str | None = None
        self._last_cycle_ms: float | None = None
        self._last_error: str | None = None
        self._report: dict[str, Any] = self._empty_report()
        self._activity: list[dict[str, Any]] = []
        self._perf_cache: dict[tuple[str, str, str, str], dict[str, Any]] = {}

    def _empty_report(self) -> dict[str, Any]:
        return {
            "generated_at": _utcnow_iso(),
            "summary": {
                "running": False,
                "cycle_count": 0,
                "accepted": 0,
                "rejected": 0,
                "universe_size": len(self._config.universe),
                "timeframes": list(self._config.timeframes),
            },
            "criteria": self.criteria_catalog(),
            "candidates": [],
            "rejections": [],
            "activity": [],
            "current_work": {
                "symbol": None,
                "timeframe": None,
                "step": "inactivo",
            },
        }

    def criteria_catalog(self) -> list[dict[str, Any]]:
        items = []
        for key in METHOD_ORDER + ["relative_strength_overlay", "multi_timeframe_confirmation"]:
            raw = EVIDENCE_LIBRARY[key]
            items.append(
                {
                    "key": key,
                    "label": raw["label"],
                    "family": raw["family"],
                    "description": raw["description"],
                    "evidence_score": round(float(raw["evidence_score"]) * 100, 1),
                    "sources": raw["sources"],
                }
            )
        return items

    def _log(self, level: str, message: str, **context: Any) -> None:
        entry = {
            "timestamp": _utcnow_iso(),
            "level": level,
            "message": message,
            **context,
        }
        with self._state_lock:
            self._activity.append(entry)
            self._activity = self._activity[-self._config.activity_limit :]

    def _feed_for_config(self) -> MarketFeed:
        if self._feed is None or self._feed.source != self._config.source:
            self._feed = MarketFeed(source=self._config.source)  # type: ignore[arg-type]
        return self._feed

    def _ml_strategy_for(self):
        if self._ml_strategy is None:
            try:
                from models.signals import MLSignalStrategy

                self._ml_strategy = MLSignalStrategy(
                    "scanner_ml_rf",
                    list(self._config.universe),
                    timeframe="1h",
                    model_name="rf",
                    confidence_threshold=0.55,
                    target_bars=5,
                )
            except Exception:
                logger.exception("No se pudo cargar la estrategia ML para el escáner")
                self._ml_strategy = False
        return None if self._ml_strategy is False else self._ml_strategy

    async def start(self) -> None:
        async with self._loop_lock:
            if self._task and not self._task.done():
                return
            if not self._config.enabled:
                return
            self._task = asyncio.create_task(self._loop(), name="quant-opportunity-scanner")

    async def stop(self) -> None:
        async with self._loop_lock:
            task = self._task
            self._task = None
            self._running = False
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    async def run_once_async(self) -> dict[str, Any]:
        return await asyncio.to_thread(self._run_cycle)

    async def control(self, action: str) -> dict[str, Any]:
        action = str(action or "").lower().strip()
        if action == "start":
            self._config.enabled = True
            await self.start()
            self._log("info", "Escáner iniciado por API")
        elif action == "stop":
            self._config.enabled = False
            await self.stop()
            self._log("warn", "Escáner detenido por API")
        elif action == "run_once":
            await self.run_once_async()
        else:
            raise ValueError(f"Acción de escáner no soportada: {action}")
        return self.report()

    async def _loop(self) -> None:
        while self._config.enabled:
            try:
                await self.run_once_async()
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("Fallo en el ciclo del escáner")
            await asyncio.sleep(max(int(self._config.scan_interval_sec), 15))

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "generated_at": _utcnow_iso(),
                "running": self._running,
                "cycle_count": self._cycle_count,
                "last_cycle_at": self._last_cycle_at,
                "last_cycle_ms": self._last_cycle_ms,
                "current_symbol": self._current_symbol,
                "current_timeframe": self._current_timeframe,
                "current_step": self._current_step,
                "last_error": self._last_error,
                "config": self._config.to_dict(),
                "summary": dict(self._report.get("summary") or {}),
            }

    def report(self, activity_limit: int = 60) -> dict[str, Any]:
        with self._state_lock:
            data = dict(self._report or self._empty_report())
            data["status"] = self.status()
            data["activity"] = list(self._activity[-max(1, activity_limit) :])
            return data

    def update_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._state_lock:
            if "enabled" in payload:
                self._config.enabled = bool(payload["enabled"])
            if "source" in payload:
                source = str(payload["source"] or "").strip().lower()
                self._config.source = source if source in {"yfinance", "ccxt"} else self._config.source
                self._feed = None
            if "scan_interval_sec" in payload:
                self._config.scan_interval_sec = max(15, min(int(payload["scan_interval_sec"]), 3600))
            if "min_signal_strength" in payload:
                self._config.min_signal_strength = max(0.0, min(float(payload["min_signal_strength"]), 1.0))
            if "min_local_win_rate_pct" in payload:
                self._config.min_local_win_rate_pct = max(0.0, min(float(payload["min_local_win_rate_pct"]), 100.0))
            if "min_selection_score" in payload:
                self._config.min_selection_score = max(0.0, min(float(payload["min_selection_score"]), 100.0))
            if "max_candidates" in payload:
                self._config.max_candidates = max(1, min(int(payload["max_candidates"]), 50))
            if "require_higher_tf_confirmation" in payload:
                self._config.require_higher_tf_confirmation = bool(payload["require_higher_tf_confirmation"])
            if "notes" in payload:
                self._config.notes = str(payload["notes"] or "").strip()
            if "universe" in payload:
                raw = payload["universe"]
                items = [item.strip().upper() for item in raw.split(",")] if isinstance(raw, str) else [str(item).strip().upper() for item in raw]
                items = [item for item in items if item]
                if items:
                    self._config.universe = items[:50]
            if "timeframes" in payload:
                raw = payload["timeframes"]
                items = [item.strip() for item in raw.split(",")] if isinstance(raw, str) else [str(item).strip() for item in raw]
                valid = [item for item in items if item in TIMEFRAME_ORDER]
                if valid:
                    self._config.timeframes = _timeframes_sorted(valid)
        self._log("info", "Configuración del escáner actualizada", config=self._config.to_dict())
        return self.report()

    def _load_universe_frames(self) -> dict[str, dict[str, pd.DataFrame]]:
        frames: dict[str, dict[str, pd.DataFrame]] = {}
        feed = self._feed_for_config()
        for symbol in self._config.universe:
            frames[symbol] = {}
            for timeframe in self._config.timeframes:
                self._current_symbol = symbol
                self._current_timeframe = timeframe
                self._current_step = "descargando"
                try:
                    df = feed.ohlcv(symbol, timeframe=timeframe, limit=420)
                    if df is None or df.empty:
                        raise ValueError("sin datos")
                    df = df.dropna(subset=["open", "high", "low", "close"]).copy()
                    frames[symbol][timeframe] = df
                except Exception as exc:
                    self._log(
                        "warn",
                        "No se pudo cargar temporalidad",
                        symbol=symbol,
                        timeframe=timeframe,
                        reason=str(exc),
                    )
        return frames

    def _relative_strength_percentiles(self, frames: dict[str, dict[str, pd.DataFrame]]) -> dict[str, float]:
        scores: list[tuple[str, float]] = []
        for symbol, items in frames.items():
            df = items.get("1d")
            if df is None or len(df) < 90:
                continue
            close = df["close"]
            fast = _pct_change(float(close.iloc[-1]), float(close.iloc[-21]))
            slow = _pct_change(float(close.iloc[-1]), float(close.iloc[-63]))
            scores.append((symbol, (fast * 0.45) + (slow * 0.55)))
        if not scores:
            return {symbol: 50.0 for symbol in frames}
        ordered = sorted(scores, key=lambda item: item[1])
        denom = max(len(ordered) - 1, 1)
        percentiles = {symbol: round(idx / denom * 100.0, 2) for idx, (symbol, _) in enumerate(ordered)}
        for symbol in frames:
            percentiles.setdefault(symbol, 50.0)
        return percentiles

    def _method_signal(self, method: str, df: pd.DataFrame, symbol: str, timeframe: str) -> dict[str, Any]:
        if len(df) < 60:
            return {"direction": 0, "strength": 0.0, "reason": "insufficient_data"}
        price = float(df["close"].iloc[-1])
        if method == "trend_ema_stack":
            if len(df) < 220:
                return {"direction": 0, "strength": 0.0, "reason": "ema_stack_needs_more_history"}
            ema20 = df["close"].ewm(span=20, adjust=False).mean()
            ema50 = df["close"].ewm(span=50, adjust=False).mean()
            ema200 = df["close"].ewm(span=200, adjust=False).mean()
            slope = _pct_change(float(ema20.iloc[-1]), float(ema20.iloc[-6]))
            bullish = price > float(ema20.iloc[-1]) > float(ema50.iloc[-1]) > float(ema200.iloc[-1]) and slope > 0
            bearish = price < float(ema20.iloc[-1]) < float(ema50.iloc[-1]) < float(ema200.iloc[-1]) and slope < 0
            if not bullish and not bearish:
                return {"direction": 0, "strength": 0.0, "reason": "sin_apilamiento_tendencial"}
            strength = 0.58 + min(abs(slope) / 2.0, 0.17) + min(abs(_pct_change(float(ema20.iloc[-1]), float(ema50.iloc[-1]))) / 10.0, 0.18)
            return {
                "direction": 1 if bullish else -1,
                "strength": min(strength, 0.96),
                "reasons": [
                    f"precio {'encima' if bullish else 'debajo'} de EMA20/EMA50/EMA200",
                    f"pendiente EMA20 {slope:.2f}%",
                ],
                "price": price,
            }
        if method == "breakout_donchian":
            channel_high = df["high"].shift(1).rolling(20).max()
            channel_low = df["low"].shift(1).rolling(20).min()
            atr14 = _atr(df, 14)
            vol20 = df["volume"].rolling(20).mean() if "volume" in df.columns else pd.Series(0.0, index=df.index)
            if price > float(channel_high.iloc[-1]) and float(df["volume"].iloc[-1]) >= float(vol20.iloc[-1]) * 0.9:
                breakout = price - float(channel_high.iloc[-1])
                strength = 0.55 + min((breakout / max(float(atr14.iloc[-1]), 1e-6)) * 0.12, 0.35)
                return {
                    "direction": 1,
                    "strength": min(strength, 0.94),
                    "reasons": ["ruptura del máximo de 20 barras", f"atr14 {float(atr14.iloc[-1]):.2f}"],
                    "price": price,
                }
            if price < float(channel_low.iloc[-1]) and float(df["volume"].iloc[-1]) >= float(vol20.iloc[-1]) * 0.9:
                breakout = float(channel_low.iloc[-1]) - price
                strength = 0.55 + min((breakout / max(float(atr14.iloc[-1]), 1e-6)) * 0.12, 0.35)
                return {
                    "direction": -1,
                    "strength": min(strength, 0.94),
                    "reasons": ["ruptura del mínimo de 20 barras", f"atr14 {float(atr14.iloc[-1]):.2f}"],
                    "price": price,
                }
            return {"direction": 0, "strength": 0.0, "reason": "sin_breakout_confirmado"}
        if method == "rsi_pullback_trend":
            if len(df) < 220:
                return {"direction": 0, "strength": 0.0, "reason": "rsi_pullback_needs_more_history"}
            ema50 = df["close"].ewm(span=50, adjust=False).mean()
            ema200 = df["close"].ewm(span=200, adjust=False).mean()
            rsi2 = _rsi(df["close"], 2)
            bullish = price > float(ema50.iloc[-1]) > float(ema200.iloc[-1])
            bearish = price < float(ema50.iloc[-1]) < float(ema200.iloc[-1])
            if bullish and float(rsi2.iloc[-2]) < 10 <= float(rsi2.iloc[-1]):
                strength = 0.56 + min((float(rsi2.iloc[-1]) - float(rsi2.iloc[-2])) / 100.0, 0.24)
                return {
                    "direction": 1,
                    "strength": min(strength, 0.88),
                    "reasons": ["pullback corto resuelto al alza dentro de tendencia", f"rsi2 {float(rsi2.iloc[-1]):.1f}"],
                    "price": price,
                }
            if bearish and float(rsi2.iloc[-2]) > 90 >= float(rsi2.iloc[-1]):
                strength = 0.56 + min((float(rsi2.iloc[-2]) - float(rsi2.iloc[-1])) / 100.0, 0.24)
                return {
                    "direction": -1,
                    "strength": min(strength, 0.88),
                    "reasons": ["pullback corto resuelto a la baja dentro de tendencia", f"rsi2 {float(rsi2.iloc[-1]):.1f}"],
                    "price": price,
                }
            return {"direction": 0, "strength": 0.0, "reason": "sin_pullback_util"}
        if method == "ml_directional":
            strategy = self._ml_strategy_for()
            if strategy is None:
                return {"direction": 0, "strength": 0.0, "reason": "modelo_ml_no_disponible"}
            try:
                signal = strategy.generate_signal(df, symbol)
            except Exception as exc:
                return {"direction": 0, "strength": 0.0, "reason": f"ml_error:{exc}"}
            direction = 1 if signal.signal.value == "BUY" else -1 if signal.signal.value == "SELL" else 0
            if direction == 0:
                return {"direction": 0, "strength": 0.0, "reason": signal.metadata.get("reason", "ml_hold")}
            return {
                "direction": direction,
                "strength": max(0.0, min(float(signal.confidence), 0.98)),
                "reasons": [f"modelo {signal.metadata.get('model', 'rf')} emitió {signal.signal.value}"],
                "price": price,
            }
        return {"direction": 0, "strength": 0.0, "reason": "metodo_desconocido"}

    def _backtest_method_recent(self, method: str, df: pd.DataFrame, symbol: str, timeframe: str) -> dict[str, Any]:
        last_ts = str(df.index[-1])
        cache_key = (symbol, timeframe, method, last_ts)
        cached = self._perf_cache.get(cache_key)
        if cached:
            return cached
        horizon = HORIZON_BARS.get(timeframe, 4)
        warmup = {"trend_ema_stack": 210, "breakout_donchian": 50, "rsi_pullback_trend": 210, "ml_directional": 80}.get(method, 80)
        wins = losses = sample = 0
        pnl_pos = pnl_neg = 0.0
        lookback_start = max(warmup, len(df) - 140)
        for idx in range(lookback_start, len(df) - horizon):
            window = df.iloc[: idx + 1]
            signal = self._method_signal(method, window, symbol, timeframe)
            direction = int(signal.get("direction") or 0)
            if direction == 0:
                continue
            now_price = float(window["close"].iloc[-1])
            fut_price = float(df["close"].iloc[idx + horizon])
            signed_move = ((fut_price / now_price) - 1.0) * direction
            sample += 1
            if signed_move > 0:
                wins += 1
                pnl_pos += signed_move
            else:
                losses += 1
                pnl_neg += abs(signed_move)
        result = {
            "sample": sample,
            "wins": wins,
            "losses": losses,
            "win_rate_pct": round((wins / sample) * 100.0, 2) if sample else 0.0,
            "profit_factor": round(pnl_pos / pnl_neg, 3) if pnl_neg > 0 else (999.0 if pnl_pos > 0 else 0.0),
        }
        self._perf_cache = {key: value for key, value in self._perf_cache.items() if key[:3] != (symbol, timeframe, method)}
        self._perf_cache[cache_key] = result
        return result

    def _timeframe_consensus(self, evaluations: list[dict[str, Any]]) -> dict[str, Any]:
        bull = bear = 0.0
        for item in evaluations:
            weight = float(item.get("strength") or 0.0) * float(item.get("evidence_score") or 0.0)
            if int(item.get("direction") or 0) > 0:
                bull += weight
            elif int(item.get("direction") or 0) < 0:
                bear += weight
        total = bull + bear
        if total <= 0 or bull == bear:
            return {"direction": 0, "label": "neutral", "confidence": 0.0}
        direction = 1 if bull > bear else -1
        confidence = abs(bull - bear) / total
        return {"direction": direction, "label": _direction_label(direction), "confidence": round(confidence, 4)}

    def _predicted_move_pct(self, df: pd.DataFrame, strength: float) -> float:
        atr14 = _atr(df, 14)
        atr_pct = _safe_float((float(atr14.iloc[-1]) / max(float(df["close"].iloc[-1]), 1e-6)) * 100.0)
        return round(max(atr_pct * (0.8 + strength), 0.05), 2)

    def _evaluate_symbol_timeframes(
        self,
        symbol: str,
        tf_frames: dict[str, pd.DataFrame],
        relative_strength_pct: float,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        accepted: list[dict[str, Any]] = []
        rejected: list[dict[str, Any]] = []
        consensus_map: dict[str, dict[str, Any]] = {}
        for timeframe in _timeframes_sorted(list(tf_frames)):
            df = tf_frames.get(timeframe)
            if df is None or df.empty:
                rejected.append({"symbol": symbol, "timeframe": timeframe, "reason": "sin datos cargados", "stage": "load"})
                continue
            self._current_symbol = symbol
            self._current_timeframe = timeframe
            self._current_step = "evaluando_criterios"
            evaluations: list[dict[str, Any]] = []
            for method in METHOD_ORDER:
                raw = self._method_signal(method, df, symbol, timeframe)
                if int(raw.get("direction") or 0) == 0 or float(raw.get("strength") or 0.0) < self._config.min_signal_strength:
                    continue
                perf = self._backtest_method_recent(method, df, symbol, timeframe)
                evidence = EVIDENCE_LIBRARY[method]
                evaluations.append({
                    **raw,
                    "method": method,
                    "method_label": evidence["label"],
                    "family": evidence["family"],
                    "evidence_score": evidence["evidence_score"],
                    "local_win_rate_pct": perf["win_rate_pct"],
                    "local_profit_factor": perf["profit_factor"],
                    "local_sample": perf["sample"],
                })
            consensus_map[timeframe] = self._timeframe_consensus(evaluations)
            if not evaluations:
                rejected.append({"symbol": symbol, "timeframe": timeframe, "reason": "ningún criterio produjo setup fuerte", "stage": "signal"})
                continue
            best = sorted(evaluations, key=lambda item: (float(item["local_win_rate_pct"]), float(item["strength"]), float(item["evidence_score"])), reverse=True)[0]
            higher_tf = HIGHER_TIMEFRAME_MAP.get(timeframe)
            higher_consensus = consensus_map.get(higher_tf or "", {"direction": 0, "label": "neutral", "confidence": 0.0})
            alignment_score = 55.0
            rejection_reasons: list[str] = []
            if higher_tf:
                if int(higher_consensus.get("direction") or 0) == int(best["direction"]):
                    alignment_score = round(70 + float(higher_consensus.get("confidence") or 0.0) * 30, 2)
                elif int(higher_consensus.get("direction") or 0) == 0:
                    alignment_score = 45.0
                    if self._config.require_higher_tf_confirmation:
                        rejection_reasons.append(f"sin confirmación clara en {higher_tf}")
                else:
                    alignment_score = 0.0
                    rejection_reasons.append(f"{higher_tf} contradice la dirección {_direction_label(int(best['direction']))}")
            if float(best["local_win_rate_pct"]) < self._config.min_local_win_rate_pct:
                rejection_reasons.append(f"win rate local {best['local_win_rate_pct']:.2f}% < mínimo {self._config.min_local_win_rate_pct:.2f}%")
            selection_score = round(
                (float(best["local_win_rate_pct"]) * 0.42)
                + (float(relative_strength_pct) * 0.18)
                + (float(best["strength"]) * 100.0 * 0.18)
                + (alignment_score * 0.14)
                + (float(best["evidence_score"]) * 100.0 * 0.08),
                2,
            )
            if selection_score < self._config.min_selection_score:
                rejection_reasons.append(f"selection score {selection_score:.2f} < mínimo {self._config.min_selection_score:.2f}")
            reasons = list(best.get("reasons") or [])
            reasons.append(f"fuerza relativa {relative_strength_pct:.1f}% del universo")
            if higher_tf:
                reasons.append(f"confirmación {higher_tf}: {higher_consensus.get('label', 'neutral')}")
            if rejection_reasons:
                rejected.append({
                    "symbol": symbol,
                    "timeframe": timeframe,
                    "direction": _direction_label(int(best["direction"])),
                    "method": best["method"],
                    "method_label": best["method_label"],
                    "local_win_rate_pct": best["local_win_rate_pct"],
                    "selection_score": selection_score,
                    "reasons": rejection_reasons,
                    "why": reasons,
                    "stage": "gates",
                })
                self._log("info", "Activo descartado por filtros", symbol=symbol, timeframe=timeframe, method=best["method"], reasons=rejection_reasons)
                continue
            accepted.append({
                "symbol": symbol,
                "timeframe": timeframe,
                "direction": _direction_label(int(best["direction"])),
                "price": round(float(best.get("price") or df["close"].iloc[-1]), 4),
                "strategy_key": best["method"],
                "strategy_label": best["method_label"],
                "strategy_family": best["family"],
                "selection_score": selection_score,
                "signal_strength_pct": round(float(best["strength"]) * 100.0, 2),
                "local_win_rate_pct": best["local_win_rate_pct"],
                "local_profit_factor": best["local_profit_factor"],
                "local_sample": best["local_sample"],
                "predicted_move_pct": self._predicted_move_pct(df, float(best["strength"])),
                "relative_strength_pct": round(float(relative_strength_pct), 2),
                "alignment_score": alignment_score,
                "confirmation": {
                    "higher_timeframe": higher_tf,
                    "direction": higher_consensus.get("label", "neutral"),
                    "confidence_pct": round(float(higher_consensus.get("confidence") or 0.0) * 100.0, 2),
                },
                "why_selected": reasons,
                "evidence": EVIDENCE_LIBRARY[best["method"]]["sources"],
                "scan_timestamp": _utcnow_iso(),
            })
            self._log("info", "Activo seleccionado por el escáner", symbol=symbol, timeframe=timeframe, strategy=best["method"], score=selection_score, win_rate=best["local_win_rate_pct"])
        return accepted, rejected

    def _run_cycle(self) -> dict[str, Any]:
        start = time.perf_counter()
        with self._state_lock:
            self._running = True
            self._current_step = "iniciando"
            self._last_error = None
        try:
            frames = self._load_universe_frames()
            relative_strength = self._relative_strength_percentiles(frames)
            accepted: list[dict[str, Any]] = []
            rejected: list[dict[str, Any]] = []
            for symbol, tf_frames in frames.items():
                self._current_symbol = symbol
                self._current_timeframe = None
                self._current_step = "resolviendo_símbolo"
                symbol_accept, symbol_reject = self._evaluate_symbol_timeframes(symbol, tf_frames, relative_strength.get(symbol, 50.0))
                accepted.extend(symbol_accept)
                rejected.extend(symbol_reject)
            accepted = sorted(accepted, key=lambda item: float(item["selection_score"]), reverse=True)[: self._config.max_candidates]
            cycle_ms = round((time.perf_counter() - start) * 1000.0, 2)
            with self._state_lock:
                self._cycle_count += 1
                self._last_cycle_at = _utcnow_iso()
                self._last_cycle_ms = cycle_ms
                self._current_step = "esperando_siguiente_ciclo"
                self._report = {
                    "generated_at": self._last_cycle_at,
                    "summary": {
                        "running": True,
                        "cycle_count": self._cycle_count,
                        "accepted": len(accepted),
                        "rejected": len(rejected),
                        "universe_size": len(self._config.universe),
                        "timeframes": list(self._config.timeframes),
                        "scan_interval_sec": self._config.scan_interval_sec,
                        "last_cycle_ms": cycle_ms,
                    },
                    "criteria": self.criteria_catalog(),
                    "candidates": accepted,
                    "rejections": rejected[:40],
                    "activity": list(self._activity[-20:]),
                    "current_work": {"symbol": self._current_symbol, "timeframe": self._current_timeframe, "step": self._current_step},
                }
            self._log("info", "Ciclo del escáner completado", accepted=len(accepted), rejected=len(rejected), cycle_ms=cycle_ms)
            return self.report()
        except Exception as exc:
            logger.exception("Ciclo del escáner falló")
            with self._state_lock:
                self._last_error = str(exc)
                self._report = {**self._empty_report(), "generated_at": _utcnow_iso(), "error": str(exc)}
            self._log("error", "Ciclo del escáner falló", error=str(exc))
            return self.report()
        finally:
            with self._state_lock:
                self._running = False
