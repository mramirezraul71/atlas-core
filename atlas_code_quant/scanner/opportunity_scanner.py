"""Atlas Code-Quant — Escáner permanente de oportunidades explicable."""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from math import isfinite
from typing import Any

import pandas as pd

from backtesting.winning_probability import TradierClient
from config.settings import settings
from data.feed import MarketFeed
from learning.adaptive_policy import AdaptiveLearningService
from scanner.universe_catalog import ScannerUniverseCatalog
from scanner.asset_classifier import classify_asset

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
    "order_flow_proxy": {
        "label": "Order flow intradía",
        "family": "microestructura",
        "evidence_score": 0.79,
        "description": "Usa presión neta de volumen, precio vs VWAP y desequilibrio bid/ask como filtro microestructural.",
        "sources": [
            {
                "title": "Get Time & Sales",
                "url": "https://docs.tradier.com/reference/brokerage-api-markets-get-timesales",
                "note": "Tradier expone time and sales para construir presión intradía y proxy de order flow.",
            },
            {
                "title": "Order Flow and Exchange Rate Dynamics",
                "url": "https://www.nber.org/papers/w12682.pdf",
                "note": "Documenta poder predictivo del order flow sobre movimientos futuros de precios.",
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


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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
    min_local_profit_factor: float
    min_backtest_sample: int
    min_selection_score: float
    max_candidates: int
    require_higher_tf_confirmation: bool
    universe_mode: str
    universe_batch_size: int
    prefilter_count: int
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
            "min_local_profit_factor": self.min_local_profit_factor,
            "min_backtest_sample": self.min_backtest_sample,
            "min_selection_score": self.min_selection_score,
            "max_candidates": self.max_candidates,
            "require_higher_tf_confirmation": self.require_higher_tf_confirmation,
            "universe_mode": self.universe_mode,
            "universe_batch_size": self.universe_batch_size,
            "prefilter_count": self.prefilter_count,
            "universe": list(self.universe),
            "timeframes": list(self.timeframes),
            "activity_limit": self.activity_limit,
            "notes": self.notes,
        }


class OpportunityScannerService:
    """Escáner continuo con explicación explícita y criterios gobernados."""

    def __init__(self, learning: AdaptiveLearningService | None = None) -> None:
        self._config = ScannerConfig(
            enabled=settings.scanner_enabled,
            source=settings.scanner_source,
            scan_interval_sec=settings.scanner_scan_interval_sec,
            min_signal_strength=settings.scanner_min_signal_strength,
            min_local_win_rate_pct=settings.scanner_min_local_win_rate_pct,
            min_local_profit_factor=settings.scanner_min_local_profit_factor,
            min_backtest_sample=settings.scanner_min_backtest_sample,
            min_selection_score=settings.scanner_min_selection_score,
            max_candidates=settings.scanner_max_candidates,
            require_higher_tf_confirmation=settings.scanner_require_higher_tf_confirmation,
            universe_mode=settings.scanner_universe_mode,
            universe_batch_size=settings.scanner_universe_batch_size,
            prefilter_count=settings.scanner_prefilter_count,
            universe=list(settings.scanner_universe),
            timeframes=_timeframes_sorted(list(settings.scanner_timeframes)),
            activity_limit=settings.scanner_activity_limit,
            notes="fase paper: escáner explicable y no ejecutor",
        )
        self._feed: MarketFeed | None = None
        self._ml_strategy = None
        self.learning = learning or AdaptiveLearningService()
        self._catalog = ScannerUniverseCatalog(
            cache_path=settings.scanner_universe_cache_path,
            cache_ttl_sec=settings.scanner_universe_cache_ttl_sec,
        )
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
        self._universe_cursor = 0
        self._last_universe_meta: dict[str, Any] = {}
        self._tradier_market_client: TradierClient | None = None
        self._tradier_market_scope: str | None = None

    def _base_universe_summary(self) -> tuple[dict[str, Any], dict[str, Any]]:
        if self._config.source == "yfinance" and self._config.universe_mode == "us_equities_rotating":
            catalog = self._catalog.get_us_equities(force_refresh=False)
            total = int(catalog.get("total_symbols") or len(catalog.get("symbols") or []))
            batch_size = min(self._config.universe_batch_size, total) if total else 0
            selected = min(self._config.prefilter_count, batch_size) if batch_size else 0
            universe_meta = {
                "mode": "us_equities_rotating",
                "universe_total": total,
                "batch_size": batch_size,
                "batch_start": 0,
                "batch_end": batch_size,
                "catalog_source": catalog.get("source") or "nasdaq_trader_symbol_directory",
                "catalog_fetched_at": catalog.get("fetched_at"),
                "prefilter_scored": 0,
                "prefilter_selected": selected,
                "selected_symbols": [],
            }
            summary = {
                "universe_size": total,
                "universe_mode": "us_equities_rotating",
                "universe_total": total,
                "batch_size": batch_size,
                "batch_start": 0,
                "batch_end": batch_size,
                "prefilter_selected": selected,
                "prefilter_scored": 0,
                "deep_scan_symbols": selected,
                "provisional_candidates": 0,
                "dynamic_min_selection_score": float(self._config.min_selection_score),
            }
            return summary, universe_meta

        manual_count = len(self._config.universe)
        summary = {
            "universe_size": manual_count,
            "universe_mode": self._config.universe_mode,
            "universe_total": manual_count,
            "batch_size": manual_count,
            "batch_start": 0,
            "batch_end": manual_count,
            "prefilter_selected": manual_count,
            "prefilter_scored": manual_count,
            "deep_scan_symbols": manual_count,
            "provisional_candidates": 0,
            "dynamic_min_selection_score": float(self._config.min_selection_score),
        }
        universe_meta = {
            "mode": "manual",
            "universe_total": manual_count,
            "batch_size": manual_count,
            "batch_start": 0,
            "batch_end": manual_count,
            "catalog_source": "manual",
            "prefilter_scored": manual_count,
            "prefilter_selected": manual_count,
            "selected_symbols": list(self._config.universe),
        }
        return summary, universe_meta

    def _empty_report(self) -> dict[str, Any]:
        summary_defaults, universe_defaults = self._base_universe_summary()
        return {
            "generated_at": _utcnow_iso(),
            "summary": {
                "running": False,
                "cycle_count": 0,
                "accepted": 0,
                "rejected": 0,
                **summary_defaults,
                "timeframes": list(self._config.timeframes),
            },
            "criteria": self.criteria_catalog(),
            "candidates": [],
            "rejections": [],
            "activity": [],
            "universe": universe_defaults,
            "current_work": {
                "symbol": None,
                "timeframe": None,
                "step": "inactivo",
            },
            "learning": self.learning.status(account_scope=settings.tradier_default_scope),
        }

    def criteria_catalog(self) -> list[dict[str, Any]]:
        items = []
        for key in METHOD_ORDER + ["relative_strength_overlay", "order_flow_proxy", "multi_timeframe_confirmation"]:
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

    def _market_data_scope(self) -> str:
        if settings.tradier_live_token:
            return "live"
        return settings.tradier_default_scope if settings.tradier_default_scope in {"live", "paper"} else "paper"

    def _tradier_client_for_market_data(self) -> TradierClient | None:
        scope = self._market_data_scope()
        if self._tradier_market_client is not None and self._tradier_market_scope == scope:
            return self._tradier_market_client
        try:
            self._tradier_market_client = TradierClient(scope=scope)
            self._tradier_market_scope = scope
        except Exception:
            logger.exception("No se pudo inicializar Tradier para order flow")
            self._tradier_market_client = None
            self._tradier_market_scope = None
        return self._tradier_market_client

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
            with self._state_lock:
                self._current_symbol = None
                self._current_timeframe = (self._config.timeframes or [None])[0]
                self._current_step = "arrancando"
                self._last_error = None
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

    def _status_payload_locked(self) -> dict[str, Any]:
        task_alive = bool(self._task and not self._task.done())
        runtime_running = bool(self._running or task_alive)
        summary = dict(self._report.get("summary") or {})
        summary["running"] = runtime_running
        summary["cycle_count"] = self._cycle_count
        summary["timeframes"] = list(self._config.timeframes)
        return {
            "generated_at": _utcnow_iso(),
            "running": runtime_running,
            "cycle_count": self._cycle_count,
            "last_cycle_at": self._last_cycle_at,
            "last_cycle_ms": self._last_cycle_ms,
            "current_symbol": self._current_symbol,
            "current_timeframe": self._current_timeframe,
            "current_step": self._current_step,
            "last_error": self._last_error,
            "config": self._config.to_dict(),
            "summary": summary,
            "learning": self.learning.status(account_scope=settings.tradier_default_scope),
        }

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            return self._status_payload_locked()

    def report(self, activity_limit: int = 60) -> dict[str, Any]:
        with self._state_lock:
            data = dict(self._report or self._empty_report())
            status = self._status_payload_locked()
            data["status"] = status
            data["summary"] = {
                **dict(data.get("summary") or {}),
                "running": status.get("running", False),
                "cycle_count": status.get("cycle_count", 0),
                "timeframes": list(self._config.timeframes),
            }
            data["current_work"] = {
                **dict(data.get("current_work") or {}),
                "symbol": status.get("current_symbol"),
                "timeframe": status.get("current_timeframe"),
                "step": status.get("current_step"),
            }
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
            if "min_local_profit_factor" in payload:
                self._config.min_local_profit_factor = max(0.5, min(float(payload["min_local_profit_factor"]), 10.0))
            if "min_backtest_sample" in payload:
                self._config.min_backtest_sample = max(3, min(int(payload["min_backtest_sample"]), 200))
            if "min_selection_score" in payload:
                self._config.min_selection_score = max(50.0, min(float(payload["min_selection_score"]), 100.0))
            if "max_candidates" in payload:
                self._config.max_candidates = max(1, min(int(payload["max_candidates"]), 50))
            if "require_higher_tf_confirmation" in payload:
                self._config.require_higher_tf_confirmation = bool(payload["require_higher_tf_confirmation"])
            if "universe_mode" in payload:
                universe_mode = str(payload["universe_mode"] or "").strip().lower()
                if universe_mode in {"manual", "us_equities_rotating"}:
                    self._config.universe_mode = universe_mode
                    self._universe_cursor = 0
            if "universe_batch_size" in payload:
                self._config.universe_batch_size = max(20, min(int(payload["universe_batch_size"]), 500))
            if "prefilter_count" in payload:
                self._config.prefilter_count = max(8, min(int(payload["prefilter_count"]), 80))
            self._config.prefilter_count = min(self._config.prefilter_count, self._config.universe_batch_size)
            if "notes" in payload:
                self._config.notes = str(payload["notes"] or "").strip()
            if "universe" in payload:
                raw = payload["universe"]
                items = [item.strip().upper() for item in raw.split(",")] if isinstance(raw, str) else [str(item).strip().upper() for item in raw]
                items = [item for item in items if item]
                if items:
                    self._config.universe = items[:500]
            if "timeframes" in payload:
                raw = payload["timeframes"]
                items = [item.strip() for item in raw.split(",")] if isinstance(raw, str) else [str(item).strip() for item in raw]
                valid = [item for item in items if item in TIMEFRAME_ORDER]
                if valid:
                    self._config.timeframes = _timeframes_sorted(valid)
        self._log("info", "Configuración del escáner actualizada", config=self._config.to_dict())
        return self.report()

    def _resolve_scan_batch(self) -> tuple[list[str], dict[str, Any]]:
        if self._config.source != "yfinance" or self._config.universe_mode == "manual":
            symbols = list(self._config.universe)
            meta = {
                "mode": "manual",
                "universe_total": len(symbols),
                "batch_size": len(symbols),
                "batch_start": 0,
                "batch_end": len(symbols),
                "catalog_source": "manual",
            }
            self._last_universe_meta = meta
            return symbols, meta

        catalog = self._catalog.get_us_equities()
        symbols = list(catalog.get("symbols") or [])
        if not symbols:
            fallback = list(self._config.universe)
            meta = {
                "mode": "manual_fallback",
                "universe_total": len(fallback),
                "batch_size": len(fallback),
                "batch_start": 0,
                "batch_end": len(fallback),
                "catalog_source": "fallback_manual",
                "catalog_error": "catalogo vacio",
            }
            self._last_universe_meta = meta
            return fallback, meta

        total = len(symbols)
        batch_size = min(self._config.universe_batch_size, total)
        start = self._universe_cursor % total
        end = start + batch_size
        if end <= total:
            batch = symbols[start:end]
        else:
            batch = symbols[start:] + symbols[: end - total]
        self._universe_cursor = (start + batch_size) % total
        meta = {
            "mode": "us_equities_rotating",
            "universe_total": total,
            "batch_size": len(batch),
            "batch_start": start,
            "batch_end": min(end, total),
            "catalog_source": catalog.get("source"),
            "catalog_fetched_at": catalog.get("fetched_at"),
        }
        self._last_universe_meta = meta
        return batch, meta

    def _prefilter_batch(self, symbols: list[str]) -> tuple[list[str], dict[str, Any]]:
        symbols = [symbol for symbol in symbols if symbol]
        if not symbols:
            return [], {
                "prefilter_source": "none",
                "prefilter_scored": 0,
                "prefilter_selected": 0,
            }
        if self._config.source != "yfinance":
            selected = symbols[: self._config.prefilter_count]
            return selected, {
                "prefilter_source": "source_fallback",
                "prefilter_scored": len(selected),
                "prefilter_selected": len(selected),
            }

        # Fast-path defensivo: en universo rotativo grande, el batch OHLCV de Yahoo
        # puede bloquearse y dejar el escáner sin ciclos completados ni señales.
        # Seleccionamos una ventana determinista y dejamos el scoring fino al
        # análisis multi-timeframe posterior.
        if self._config.universe_mode == "us_equities_rotating" and len(symbols) >= 40:
            selected = symbols[: self._config.prefilter_count]
            return selected, {
                "prefilter_source": "rotating_fastpath",
                "prefilter_scored": len(selected),
                "prefilter_selected": len(selected),
            }

        self._current_symbol = None
        self._current_timeframe = "1d"
        self._current_step = "prefiltrando_universo"
        feed = self._feed_for_config()
        try:
            frames = feed.ohlcv_many(symbols, timeframe="1d", limit=120)
        except Exception as exc:
            self._log("warn", "No se pudo descargar lote diario para prefiltrado", reason=str(exc))
            selected = symbols[: self._config.prefilter_count]
            return selected, {
                "prefilter_source": "error_fallback",
                "prefilter_scored": 0,
                "prefilter_selected": len(selected),
                "prefilter_error": str(exc),
            }

        scored: list[tuple[str, float, dict[str, Any]]] = []
        min_dollar_volume = settings.scanner_prefilter_min_dollar_volume_millions * 1_000_000.0
        for symbol, df in frames.items():
            if df is None or df.empty or len(df) < 70:
                continue
            try:
                close = df["close"].astype(float)
                volume = df["volume"].astype(float)
                price = float(close.iloc[-1])
                if price < settings.scanner_prefilter_min_price:
                    continue
                avg_dollar_volume = float((close * volume).tail(20).mean())
                if avg_dollar_volume < min_dollar_volume:
                    continue
                ret_21 = abs(_pct_change(float(close.iloc[-1]), float(close.iloc[-21])))
                ret_63 = abs(_pct_change(float(close.iloc[-1]), float(close.iloc[-63])))
                atr_pct = self._predicted_move_pct(df, 0.6)
                trend_bonus = 5.0 if (float(close.iloc[-1]) > float(close.rolling(50).mean().iloc[-1])) else 0.0
                liquidity_bonus = min(avg_dollar_volume / 100_000_000.0, 1.0) * 20.0
                score = (ret_21 * 0.42) + (ret_63 * 0.28) + (atr_pct * 1.7) + liquidity_bonus + trend_bonus
                scored.append(
                    (
                        symbol,
                        round(score, 3),
                        {
                            "price": round(price, 3),
                            "avg_dollar_volume": round(avg_dollar_volume, 2),
                            "ret_21_pct": round(ret_21, 2),
                            "ret_63_pct": round(ret_63, 2),
                            "atr_pct": round(atr_pct, 2),
                        },
                    )
                )
            except Exception:
                continue

        ordered = sorted(scored, key=lambda item: item[1], reverse=True)
        selected = [symbol for symbol, _, _ in ordered[: self._config.prefilter_count]]
        if not selected:
            selected = symbols[: self._config.prefilter_count]
        meta = {
            "prefilter_source": "daily_batch",
            "prefilter_scored": len(scored),
            "prefilter_selected": len(selected),
            "prefilter_preview": [
                {"symbol": symbol, "score": score, **details}
                for symbol, score, details in ordered[: min(12, len(ordered))]
            ],
        }
        return selected, meta

    def _load_universe_frames(self, symbols: list[str]) -> dict[str, dict[str, pd.DataFrame]]:
        frames: dict[str, dict[str, pd.DataFrame]] = {}
        feed = self._feed_for_config()
        for symbol in symbols:
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

    def _order_flow_snapshot(self, symbol: str, price_hint: float = 0.0) -> dict[str, Any]:
        scope = self._market_data_scope()
        snapshot = {
            "available": False,
            "scope": scope,
            "mode": "disabled",
            "direction": "neutral",
            "score_pct": 50.0,
            "confidence_pct": 0.0,
            "net_pressure_pct": 0.0,
            "price_vs_vwap_pct": 0.0,
            "spread_bps": 0.0,
            "quote_imbalance_pct": 0.0,
            "last_price": round(price_hint, 4) if price_hint > 0 else 0.0,
            "vwap": None,
            "reason": "sin_datos",
        }
        client = self._tradier_client_for_market_data()
        if client is None:
            snapshot["reason"] = "tradier_no_disponible"
            return snapshot

        try:
            quote = client.quote(symbol)
        except Exception as exc:
            snapshot["reason"] = f"quote_error:{exc}"
            return snapshot

        bid = _safe_float(quote.get("bid"), 0.0)
        ask = _safe_float(quote.get("ask"), 0.0)
        bid_size = _safe_float(quote.get("bidsize"), _safe_float(quote.get("bid_size"), 0.0))
        ask_size = _safe_float(quote.get("asksize"), _safe_float(quote.get("ask_size"), 0.0))
        last_price = _safe_float(quote.get("last"), price_hint)
        if last_price <= 0:
            last_price = price_hint
        if bid > 0 and ask > 0:
            mid = (bid + ask) / 2.0
            snapshot["spread_bps"] = round(((ask - bid) / max(mid, 1e-6)) * 10000.0, 2)
        total_size = bid_size + ask_size
        if total_size > 0:
            snapshot["quote_imbalance_pct"] = round(((bid_size - ask_size) / total_size) * 100.0, 2)

        end = datetime.now(timezone.utc)
        start = end - timedelta(minutes=45)
        rows: list[dict[str, Any]] = []
        interval_used = "1min"
        for interval in ("1min", "5min"):
            try:
                rows = client.timesales(symbol, interval=interval, start=start, end=end, session_filter="all")
                interval_used = interval
                if rows:
                    break
            except Exception:
                rows = []
        if not rows:
            snapshot["last_price"] = round(last_price, 4) if last_price > 0 else 0.0
            snapshot["reason"] = "timesales_vacio"
            return snapshot

        df = pd.DataFrame(rows)
        if df.empty:
            snapshot["last_price"] = round(last_price, 4) if last_price > 0 else 0.0
            snapshot["reason"] = "timesales_vacio"
            return snapshot

        for column in ("close", "open", "high", "low", "price", "volume", "vwap"):
            if column in df.columns:
                df[column] = pd.to_numeric(df[column], errors="coerce")
        close_series = df["close"] if "close" in df.columns else df["price"] if "price" in df.columns else pd.Series(dtype=float)
        if close_series.empty:
            snapshot["last_price"] = round(last_price, 4) if last_price > 0 else 0.0
            snapshot["reason"] = "timesales_sin_precio"
            return snapshot
        close_series = close_series.ffill().dropna()
        if close_series.empty:
            snapshot["last_price"] = round(last_price, 4) if last_price > 0 else 0.0
            snapshot["reason"] = "timesales_sin_precio"
            return snapshot

        volume_series = pd.to_numeric(df.get("volume", pd.Series([0.0] * len(df))), errors="coerce").fillna(0.0)
        if "open" in df.columns and not df["open"].dropna().empty:
            open_series = pd.to_numeric(df["open"], errors="coerce").fillna(close_series.shift(1)).fillna(close_series)
        else:
            open_series = close_series.shift(1).fillna(close_series)
        signed_volume = volume_series * (close_series - open_series).apply(lambda value: 1.0 if value > 0 else -1.0 if value < 0 else 0.0)
        total_volume = max(float(volume_series.sum()), 1.0)
        net_pressure_pct = float(signed_volume.sum()) / total_volume * 100.0
        if "vwap" in df.columns and not df["vwap"].dropna().empty:
            vwap = _safe_float(df["vwap"].dropna().iloc[-1], 0.0)
        else:
            weighted = float((close_series * volume_series).sum())
            vwap = weighted / total_volume if total_volume > 0 else float(close_series.iloc[-1])
        last_ts_price = _safe_float(close_series.iloc[-1], last_price)
        price_vs_vwap_pct = _pct_change(last_ts_price, vwap) if vwap > 0 else 0.0

        raw_score = 50.0
        raw_score += max(min(net_pressure_pct * 0.55, 22.0), -22.0)
        raw_score += max(min(price_vs_vwap_pct * 35.0, 18.0), -18.0)
        raw_score += max(min(snapshot["quote_imbalance_pct"] * 0.18, 10.0), -10.0)
        raw_score -= min(snapshot["spread_bps"] / 12.0, 8.0)
        score_pct = max(0.0, min(raw_score, 100.0))
        if score_pct >= 55.0:
            direction = "alcista"
        elif score_pct <= 45.0:
            direction = "bajista"
        else:
            direction = "neutral"

        snapshot.update(
            {
                "available": True,
                "scope": scope,
                "mode": "proxy_intradia",
                "interval": interval_used,
                "direction": direction,
                "score_pct": round(score_pct, 2),
                "confidence_pct": round(min(abs(score_pct - 50.0) * 2.0, 100.0), 2),
                "net_pressure_pct": round(net_pressure_pct, 2),
                "price_vs_vwap_pct": round(price_vs_vwap_pct, 3),
                "last_price": round(last_ts_price, 4),
                "vwap": round(vwap, 4) if vwap > 0 else None,
                "reason": "order_flow_proxy_ok",
            }
        )
        return snapshot

    def _order_flow_batch(self, frames: dict[str, dict[str, pd.DataFrame]]) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for symbol, tf_frames in frames.items():
            price_hint = 0.0
            for tf in ("1d", "4h", "1h", "15m", "5m"):
                df = tf_frames.get(tf)
                if df is not None and not df.empty:
                    price_hint = _safe_float(df["close"].iloc[-1], 0.0)
                    if price_hint > 0:
                        break
            try:
                out[symbol] = self._order_flow_snapshot(symbol, price_hint=price_hint)
            except Exception as exc:
                out[symbol] = {
                    "available": False,
                    "scope": self._market_data_scope(),
                    "mode": "proxy_intradia",
                    "direction": "neutral",
                    "score_pct": 50.0,
                    "confidence_pct": 0.0,
                    "net_pressure_pct": 0.0,
                    "price_vs_vwap_pct": 0.0,
                    "spread_bps": 0.0,
                    "quote_imbalance_pct": 0.0,
                    "last_price": round(price_hint, 4) if price_hint > 0 else 0.0,
                    "vwap": None,
                    "reason": f"order_flow_error:{exc}",
                }
        return out

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
        win_moves: list[float] = []
        loss_moves: list[float] = []
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
                win_moves.append(signed_move)
            else:
                losses += 1
                pnl_neg += abs(signed_move)
                loss_moves.append(abs(signed_move))
        avg_win_pct = (sum(win_moves) / len(win_moves) * 100.0) if win_moves else 0.0
        avg_loss_pct = (sum(loss_moves) / len(loss_moves) * 100.0) if loss_moves else 0.0
        expectancy_pct = (((pnl_pos - pnl_neg) / sample) * 100.0) if sample else 0.0
        profit_factor = round(pnl_pos / pnl_neg, 3) if pnl_neg > 0 else (999.0 if pnl_pos > 0 else 0.0)
        payoff_ratio = round(avg_win_pct / avg_loss_pct, 3) if avg_loss_pct > 0 else (999.0 if avg_win_pct > 0 else 0.0)
        sample_confidence_pct = round(_clip((sample / 24.0) ** 0.5 * 100.0, 10.0, 100.0), 2) if sample else 0.0
        profit_factor_score_pct = round(_clip((min(profit_factor, 3.0) - 0.8) / 1.4 * 100.0, 0.0, 100.0), 2)
        expectancy_score_pct = round(_clip((expectancy_pct + 0.30) / 0.90 * 100.0, 0.0, 100.0), 2)
        payoff_score_pct = round(_clip((min(payoff_ratio, 2.5) - 0.6) / 1.4 * 100.0, 0.0, 100.0), 2)
        quality_score_pct = round(
            (float((wins / sample) * 100.0) * 0.28 if sample else 0.0)
            + (profit_factor_score_pct * 0.24)
            + (expectancy_score_pct * 0.20)
            + (payoff_score_pct * 0.14)
            + (sample_confidence_pct * 0.14),
            2,
        )
        result = {
            "sample": sample,
            "wins": wins,
            "losses": losses,
            "win_rate_pct": round((wins / sample) * 100.0, 2) if sample else 0.0,
            "profit_factor": profit_factor,
            "expectancy_pct": round(expectancy_pct, 4),
            "avg_win_pct": round(avg_win_pct, 4),
            "avg_loss_pct": round(avg_loss_pct, 4),
            "payoff_ratio": payoff_ratio,
            "sample_confidence_pct": sample_confidence_pct,
            "profit_factor_score_pct": profit_factor_score_pct,
            "expectancy_score_pct": expectancy_score_pct,
            "payoff_score_pct": payoff_score_pct,
            "quality_score_pct": quality_score_pct,
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

    def _dynamic_selection_threshold(self, candidates: list[dict[str, Any]]) -> float:
        base_floor = float(self._config.min_selection_score)
        if not candidates:
            return round(base_floor, 2)

        ordered_scores = sorted((float(item.get("selection_score") or 0.0) for item in candidates), reverse=True)
        if len(ordered_scores) <= self._config.max_candidates:
            return round(base_floor, 2)

        cutoff_index = min(len(ordered_scores) - 1, max(self._config.max_candidates - 1, 0))
        rank_cutoff = ordered_scores[cutoff_index]
        overload = max(len(ordered_scores) - self._config.max_candidates, 0)
        density_bonus = min(overload * 0.35, 10.0)
        threshold = max(base_floor, rank_cutoff, base_floor + density_bonus)
        return round(min(threshold, 99.5), 2)

    def _evaluate_symbol_timeframes(
        self,
        symbol: str,
        tf_frames: dict[str, pd.DataFrame],
        relative_strength_pct: float,
        order_flow: dict[str, Any] | None = None,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        accepted: list[dict[str, Any]] = []
        rejected: list[dict[str, Any]] = []
        consensus_map: dict[str, dict[str, Any]] = {}
        order_flow = order_flow or {}
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
                    "local_expectancy_pct": perf["expectancy_pct"],
                    "local_avg_win_pct": perf["avg_win_pct"],
                    "local_avg_loss_pct": perf["avg_loss_pct"],
                    "local_payoff_ratio": perf["payoff_ratio"],
                    "local_sample": perf["sample"],
                    "sample_confidence_pct": perf["sample_confidence_pct"],
                    "local_quality_score_pct": perf["quality_score_pct"],
                })
            consensus_map[timeframe] = self._timeframe_consensus(evaluations)
            if not evaluations:
                rejected.append({"symbol": symbol, "timeframe": timeframe, "reason": "ningún criterio produjo setup fuerte", "stage": "signal"})
                continue
            best = sorted(
                evaluations,
                key=lambda item: (
                    float(item.get("local_quality_score_pct") or 0.0),
                    float(item.get("local_profit_factor") or 0.0),
                    float(item.get("local_expectancy_pct") or 0.0),
                    float(item.get("strength") or 0.0),
                    float(item.get("evidence_score") or 0.0),
                ),
                reverse=True,
            )[0]
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
            if float(best["local_profit_factor"]) < self._config.min_local_profit_factor:
                rejection_reasons.append(
                    f"profit factor local {best['local_profit_factor']:.2f} < mínimo {self._config.min_local_profit_factor:.2f}"
                )
            if int(best["local_sample"]) < self._config.min_backtest_sample:
                rejection_reasons.append(
                    f"sample local {int(best['local_sample'])} < mínimo {int(self._config.min_backtest_sample)}"
                )
            adaptive_context = self.learning.context(
                symbol=symbol,
                direction=_direction_label(int(best.get("direction") or 0)),
                account_scope=settings.tradier_default_scope,
            )
            adaptive_bias = _safe_float(adaptive_context.get("total_bias"), 0.0)
            order_flow_score = _safe_float(order_flow.get("score_pct"), 50.0)
            order_flow_direction = str(order_flow.get("direction") or "neutral").lower()
            order_flow_confidence = _safe_float(order_flow.get("confidence_pct"), 0.0)
            order_flow_alignment_score = 50.0
            if bool(order_flow.get("available")):
                if order_flow_direction == _direction_label(int(best["direction"])):
                    order_flow_alignment_score = min(70.0 + (order_flow_confidence * 0.30), 100.0)
                elif order_flow_direction == "neutral":
                    order_flow_alignment_score = 48.0
                else:
                    order_flow_alignment_score = max(25.0 - (order_flow_confidence * 0.15), 0.0)
            local_quality_score = float(best.get("local_quality_score_pct") or 0.0)
            selection_score = round(
                (local_quality_score * 0.44)
                + (float(relative_strength_pct) * 0.14)
                + (float(best["strength"]) * 100.0 * 0.12)
                + (alignment_score * 0.10)
                + (float(best["evidence_score"]) * 100.0 * 0.06)
                + (order_flow_score * 0.08)
                + (order_flow_alignment_score * 0.06)
                + adaptive_bias,
                2,
            )
            if (
                bool(order_flow.get("available"))
                and timeframe in {"5m", "15m", "1h"}
                and order_flow_direction in {"alcista", "bajista"}
                and order_flow_direction != _direction_label(int(best["direction"]))
                and order_flow_confidence >= 70.0
            ):
                rejection_reasons.append(f"order flow {order_flow_direction} contradice el setup con {order_flow_confidence:.1f}%")
            if selection_score < self._config.min_selection_score:
                rejection_reasons.append(f"selection score {selection_score:.2f} < mínimo {self._config.min_selection_score:.2f}")
            reasons = list(best.get("reasons") or [])
            reasons.append(
                "calidad local "
                f"{local_quality_score:.1f} | pf {float(best.get('local_profit_factor') or 0.0):.2f} | "
                f"expectancy {float(best.get('local_expectancy_pct') or 0.0):.3f}% | "
                f"sample {int(best.get('local_sample') or 0)}"
            )
            reasons.append(f"fuerza relativa {relative_strength_pct:.1f}% del universo")
            if higher_tf:
                reasons.append(f"confirmación {higher_tf}: {higher_consensus.get('label', 'neutral')}")
            if bool(order_flow.get("available")):
                reasons.append(
                    f"order flow {order_flow.get('direction', 'neutral')} {order_flow_score:.1f}% · presión {float(order_flow.get('net_pressure_pct') or 0.0):.1f}%"
                )
            if adaptive_bias >= 1.0:
                reasons.append(f"aprendizaje reciente suma {adaptive_bias:.2f} pts a {symbol}")
            elif adaptive_bias <= -1.0:
                reasons.append(f"aprendizaje reciente resta {abs(adaptive_bias):.2f} pts y pide más cuidado")
            if rejection_reasons:
                rejected.append({
                    "symbol": symbol,
                    "timeframe": timeframe,
                    "direction": _direction_label(int(best["direction"])),
                    "method": best["method"],
                    "method_label": best["method_label"],
                    "local_win_rate_pct": best["local_win_rate_pct"],
                    "local_profit_factor": best["local_profit_factor"],
                    "local_expectancy_pct": best["local_expectancy_pct"],
                    "local_payoff_ratio": best["local_payoff_ratio"],
                    "local_sample": best["local_sample"],
                    "local_quality_score_pct": local_quality_score,
                    "selection_score": selection_score,
                    "adaptive_context": adaptive_context,
                    "order_flow": order_flow,
                    "reasons": rejection_reasons,
                    "why": reasons,
                    "stage": "gates",
                })
                self._log("info", "Activo descartado por filtros", symbol=symbol, timeframe=timeframe, method=best["method"], reasons=rejection_reasons)
                continue
            _asset_profile = classify_asset(symbol)
            _closes_series = df["close"].dropna().tail(200)
            _recent_closes = [round(float(x), 6) for x in _closes_series.tolist()]
            accepted.append({
                "symbol": symbol,
                "asset_class": _asset_profile.asset_class.value,
                "has_options": _asset_profile.has_options,
                "timeframe": timeframe,
                "direction": _direction_label(int(best["direction"])),
                "price": round(float(best.get("price") or df["close"].iloc[-1]), 4),
                "recent_closes": _recent_closes,
                "strategy_key": best["method"],
                "strategy_label": best["method_label"],
                "strategy_family": best["family"],
                "selection_score": selection_score,
                "adaptive_context": adaptive_context,
                "signal_strength_pct": round(float(best["strength"]) * 100.0, 2),
                "local_win_rate_pct": best["local_win_rate_pct"],
                "local_profit_factor": best["local_profit_factor"],
                "local_expectancy_pct": best["local_expectancy_pct"],
                "local_avg_win_pct": best["local_avg_win_pct"],
                "local_avg_loss_pct": best["local_avg_loss_pct"],
                "local_payoff_ratio": best["local_payoff_ratio"],
                "local_sample": best["local_sample"],
                "sample_confidence_pct": best["sample_confidence_pct"],
                "local_quality_score_pct": local_quality_score,
                "predicted_move_pct": self._predicted_move_pct(df, float(best["strength"])),
                "relative_strength_pct": round(float(relative_strength_pct), 2),
                "alignment_score": alignment_score,
                "order_flow": order_flow,
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
            batch_symbols, universe_meta = self._resolve_scan_batch()
            selected_symbols, prefilter_meta = self._prefilter_batch(batch_symbols)
            frames = self._load_universe_frames(selected_symbols)
            relative_strength = self._relative_strength_percentiles(frames)
            order_flow_batch = self._order_flow_batch(frames)
            accepted: list[dict[str, Any]] = []
            rejected: list[dict[str, Any]] = []
            for symbol, tf_frames in frames.items():
                self._current_symbol = symbol
                self._current_timeframe = None
                self._current_step = "resolviendo_símbolo"
                symbol_accept, symbol_reject = self._evaluate_symbol_timeframes(
                    symbol,
                    tf_frames,
                    relative_strength.get(symbol, 50.0),
                    order_flow_batch.get(symbol),
                )
                accepted.extend(symbol_accept)
                rejected.extend(symbol_reject)
            provisional_count = len(accepted)
            dynamic_min_selection_score = self._dynamic_selection_threshold(accepted)
            dynamically_filtered: list[dict[str, Any]] = []
            for item in accepted:
                if float(item.get("selection_score") or 0.0) >= dynamic_min_selection_score:
                    dynamically_filtered.append(item)
                    continue
                rejected.append({
                    "symbol": item.get("symbol"),
                    "timeframe": item.get("timeframe"),
                    "direction": item.get("direction"),
                    "method": item.get("strategy_key"),
                    "method_label": item.get("strategy_label"),
                    "local_win_rate_pct": item.get("local_win_rate_pct"),
                    "selection_score": item.get("selection_score"),
                    "adaptive_context": item.get("adaptive_context") or {},
                    "reasons": [f"score {float(item.get('selection_score') or 0.0):.2f} < umbral dinámico {dynamic_min_selection_score:.2f}"],
                    "why": item.get("why_selected") or [],
                    "stage": "dynamic_threshold",
                })
            accepted = sorted(dynamically_filtered, key=lambda item: float(item["selection_score"]), reverse=True)[: self._config.max_candidates]
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
                        "universe_size": int(universe_meta.get("universe_total") or len(batch_symbols)),
                        "universe_mode": universe_meta.get("mode", self._config.universe_mode),
                        "universe_total": int(universe_meta.get("universe_total") or len(batch_symbols)),
                        "batch_size": int(universe_meta.get("batch_size") or len(batch_symbols)),
                        "batch_start": int(universe_meta.get("batch_start") or 0),
                        "batch_end": int(universe_meta.get("batch_end") or len(batch_symbols)),
                        "prefilter_selected": int(prefilter_meta.get("prefilter_selected") or len(selected_symbols)),
                        "prefilter_scored": int(prefilter_meta.get("prefilter_scored") or len(selected_symbols)),
                        "deep_scan_symbols": len(selected_symbols),
                        "provisional_candidates": provisional_count,
                        "dynamic_min_selection_score": dynamic_min_selection_score,
                        "timeframes": list(self._config.timeframes),
                        "scan_interval_sec": self._config.scan_interval_sec,
                        "last_cycle_ms": cycle_ms,
                    },
                    "criteria": self.criteria_catalog(),
                    "learning": self.learning.status(account_scope=settings.tradier_default_scope),
                    "universe": {
                        **universe_meta,
                        **prefilter_meta,
                        "selected_symbols": list(selected_symbols),
                    },
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
