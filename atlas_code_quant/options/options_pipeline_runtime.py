"""Runtime scheduler para Options Pipeline multi-activo en paper mode."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Optional
from zoneinfo import ZoneInfo

from atlas_code_quant.backtesting.winning_probability import TradierClient
from atlas_code_quant.execution.option_chain_cache import OptionChainCache
from atlas_code_quant.options.options_pipeline import run_options_trading_pipeline

logger = logging.getLogger("quant.options.pipeline_runtime")

_ET = ZoneInfo("America/New_York")

OPTIONS_PIPELINE_RUNTIME_CONFIG: dict[str, Any] = {
    "enabled": True,
    "paper_only": True,
    "market_interval_seconds": 300,
    "premarket_interval_seconds": 900,
    "max_trades_per_cycle": 5,
    "dedupe_window_minutes": 30,
    "max_corr": 0.75,
}


def _env_bool(key: str, default: bool) -> bool:
    raw = str(os.environ.get(key, str(default))).strip().lower()
    return raw not in {"0", "false", "no", "off"}


def _env_int(key: str, default: int, *, lo: int = 1) -> int:
    try:
        return max(lo, int(os.environ.get(key, str(default))))
    except Exception:
        return default


def _env_float(key: str, default: float, *, lo: float = 0.0) -> float:
    try:
        return max(lo, float(os.environ.get(key, str(default))))
    except Exception:
        return default


def build_options_pipeline_runtime_config() -> dict[str, Any]:
    cfg = dict(OPTIONS_PIPELINE_RUNTIME_CONFIG)
    cfg["enabled"] = _env_bool("QUANT_OPTIONS_PIPELINE_RUNTIME_ENABLED", bool(cfg["enabled"]))
    cfg["paper_only"] = _env_bool("QUANT_OPTIONS_PIPELINE_PAPER_ONLY", bool(cfg["paper_only"]))
    cfg["market_interval_seconds"] = _env_int(
        "QUANT_OPTIONS_PIPELINE_MARKET_INTERVAL_SECONDS",
        int(cfg["market_interval_seconds"]),
        lo=30,
    )
    cfg["premarket_interval_seconds"] = _env_int(
        "QUANT_OPTIONS_PIPELINE_PREMARKET_INTERVAL_SECONDS",
        int(cfg["premarket_interval_seconds"]),
        lo=60,
    )
    cfg["max_trades_per_cycle"] = _env_int(
        "QUANT_OPTIONS_PIPELINE_MAX_TRADES_PER_CYCLE",
        int(cfg["max_trades_per_cycle"]),
        lo=1,
    )
    cfg["dedupe_window_minutes"] = _env_int(
        "QUANT_OPTIONS_PIPELINE_DEDUPE_WINDOW_MINUTES",
        int(cfg["dedupe_window_minutes"]),
        lo=1,
    )
    cfg["max_corr"] = _env_float(
        "QUANT_OPTIONS_PIPELINE_MAX_CORR",
        float(cfg["max_corr"]),
        lo=0.0,
    )
    return cfg


def options_pipeline_runtime_enabled() -> bool:
    return bool(build_options_pipeline_runtime_config().get("enabled"))


def _market_phase_et(now_utc: datetime) -> str:
    now_et = now_utc.astimezone(_ET)
    if now_et.weekday() >= 5:
        return "closed"
    hhmm = now_et.hour * 60 + now_et.minute
    if 4 * 60 <= hhmm < 9 * 60 + 30:
        return "premarket"
    if 9 * 60 + 30 <= hhmm < 16 * 60:
        return "market"
    return "closed"


class _LocalSignalDedupe:
    def __init__(self) -> None:
        self._last_seen: dict[tuple[str, str, str, str], datetime] = {}
        self._cycle_anchor: datetime | None = None

    def set_cycle_anchor(self, current_time: datetime) -> None:
        self._cycle_anchor = current_time

    def has_recent(self, symbol: str, strategy: str, expiry: str, within_minutes: int) -> bool:
        anchor = self._cycle_anchor or datetime.now(timezone.utc)
        bucket = anchor.replace(second=0, microsecond=0)
        key = (symbol.upper(), strategy.upper(), expiry, bucket.isoformat())
        ttl = timedelta(minutes=max(1, within_minutes))
        prev = self._last_seen.get(key)
        if prev is not None and (anchor - prev) <= ttl:
            return True
        self._last_seen[key] = anchor
        return False


class _AtlasDedupeWrapper:
    def __init__(self, atlas: Any, dedupe: _LocalSignalDedupe) -> None:
        self._atlas = atlas
        self._dedupe = dedupe

    def __getattr__(self, name: str) -> Any:
        return getattr(self._atlas, name)

    def set_cycle_anchor(self, current_time: datetime) -> None:
        self._dedupe.set_cycle_anchor(current_time)

    def has_recent_signal(self, symbol: str, strategy: str, expiry: str, within_minutes: int = 30) -> bool:
        fn = getattr(self._atlas, "has_recent_signal", None)
        if callable(fn):
            try:
                return bool(fn(symbol, strategy, expiry, within_minutes=within_minutes))
            except Exception:
                pass
        repeated = self._dedupe.has_recent(symbol, strategy, expiry, within_minutes)
        if repeated:
            try:
                self._atlas.log_event(
                    {
                        "event_type": "signal_skipped",
                        "symbol": symbol,
                        "strategy": strategy,
                        "reason": "duplicate_signal_local_cache",
                        "expiry": expiry,
                    }
                )
            except Exception:
                pass
        return repeated


class PaperAtlasOptionsAdapter:
    """Adapter ligero para ejecutar el pipeline en paper con bajo acoplamiento.

    Implementa la interfaz esperada por ``run_options_trading_pipeline``.
    """

    def __init__(self, *, universe: list[str] | None = None) -> None:
        self._client = TradierClient(scope="paper")
        self._chain_cache = OptionChainCache(ttl_sec=120)
        self._events: list[dict[str, Any]] = []
        self._signals: list[dict[str, Any]] = []
        self._positions: list[dict[str, Any]] = []
        base_universe = universe or ["SPY", "QQQ", "IWM", "AAPL", "MSFT", "NVDA", "AMZN", "TSLA"]
        self._universe = [str(s).strip().upper() for s in base_universe if str(s).strip()]

    def get_global_regime(self) -> dict[str, Any]:
        now = datetime.now(timezone.utc)
        phase = _market_phase_et(now)
        return {
            "event_risk": False,
            "regime_id": f"paper_{phase}",
            "tradable_today": phase in {"market", "premarket"},
        }

    def get_optionable_universe(self) -> list[str]:
        return list(self._universe)

    def filter_liquid_symbols(self, symbols: list[str]) -> list[str]:
        return list(symbols)

    def get_asset_metadata(self, symbol: str) -> dict[str, Any]:
        quote = self._client.quote(symbol)
        bid = float(quote.get("bid") or 0.0)
        ask = float(quote.get("ask") or 0.0)
        last = float(quote.get("last") or quote.get("close") or 0.0)
        mid = (bid + ask) / 2.0 if bid > 0 and ask > 0 else max(last, bid, ask, 1.0)
        spread_pct = ((ask - bid) / mid) if mid > 0 and ask >= bid > 0 else 0.02
        is_etf = symbol in {"SPY", "QQQ", "IWM", "XLF", "XLE", "XLK", "SMH", "ARKK"}
        return {
            "symbol": symbol,
            "asset_class": "ETF" if is_etf else "EQUITY",
            "sector": "broad_market" if is_etf else "technology",
            "avg_option_volume_20d": 25000.0 if is_etf else 8000.0,
            "avg_bid_ask_pct": max(0.005, min(0.03, spread_pct)),
            "avg_open_interest_20d": 20000.0 if is_etf else 9000.0,
            "has_weeklies": True,
            "earnings_date": None,
            "market_cap": 2_000_000_000_000.0 if symbol in {"AAPL", "MSFT"} else 200_000_000_000.0,
            "beta": 1.0 if is_etf else 1.35,
            "price": max(mid, 1.0),
        }

    def _expirations(self, symbol: str) -> list[str]:
        try:
            rows = self._client.expirations(symbol)
            return [str(x) for x in rows if str(x).strip()]
        except Exception:
            return []

    def get_option_chain(self, symbol: str) -> list[dict[str, Any]]:
        expirations = self._expirations(symbol)[:4]
        out: list[dict[str, Any]] = []
        for exp in expirations:
            try:
                chain = self._chain_cache.get_or_fetch(symbol, exp, self._client, scope="paper")
            except Exception:
                chain = []
            for row in chain:
                if not isinstance(row, dict):
                    continue
                greeks = row.get("greeks") if isinstance(row.get("greeks"), dict) else {}
                delta = greeks.get("delta", row.get("delta", 0.0))
                out.append(
                    {
                        "expiry": exp,
                        "type": str(row.get("option_type") or row.get("type") or "").lower(),
                        "strike": row.get("strike"),
                        "delta": delta,
                        "bid": row.get("bid"),
                        "ask": row.get("ask"),
                        "open_interest": row.get("open_interest"),
                        "volume": row.get("volume"),
                        "iv": row.get("iv") or row.get("implied_volatility") or greeks.get("mid_iv") or 0.22,
                    }
                )
        return out

    def get_vol_regime(self, symbol: str) -> dict[str, Any]:
        return {"iv_rank": 58.0, "vrp_20d": 7.0, "term_structure_slope": 0.6, "skew_25d": 7.0}

    def get_gex_surface(self, symbol: str) -> dict[str, Any]:
        return {"net_gex": -50.0, "gamma_flip_distance_pct": 1.2, "max_pain_distance_pct": 1.0}

    def get_oi_flow(self, symbol: str) -> dict[str, Any]:
        return {"oi_change_1d_pct": 11.0, "call_put_volume_ratio": 1.0, "max_pain_distance_pct": 0.9}

    def get_price_regime(self, symbol: str) -> dict[str, Any]:
        quote = self._client.quote(symbol)
        chg = float(quote.get("change_percentage") or 0.0)
        if chg >= 1.2:
            status = "up_breakout"
            ema = "bull_aligned"
        elif chg <= -1.2:
            status = "down_breakout"
            ema = "bear_aligned"
        else:
            status = "range"
            ema = "mixed"
        return {"adx": 22.0 if status == "range" else 34.0, "ema_alignment": ema, "breakout_status": status}

    def get_seasonal_multiplier(self, symbol: str, current_time: datetime) -> float:
        return 1.0

    def get_portfolio_state(self) -> dict[str, Any]:
        return {
            "open_positions": [p for p in self._positions if p.get("status") == "open"],
            "net_delta": 0.0,
            "net_vega": 0.0,
            "symbol_exposure": {},
            "sector_exposure": {},
        }

    def get_correlation_matrix(self) -> dict[str, dict[str, float]]:
        return {}

    def calculate_position_sizing(self, **kwargs: Any) -> dict[str, Any]:
        opp = kwargs.get("opportunity") or {}
        family = str(opp.get("asset_family") or "LARGE_CAP_EQUITY")
        risk_pct = 0.008
        if family == "HIGH_BETA_EQUITY":
            risk_pct = 0.0045
        elif family == "EVENT_DRIVEN_EQUITY":
            risk_pct = 0.0035
        return {"contracts": 1, "risk_pct": risk_pct, "risk_dollars": 100.0 * risk_pct}

    def emit_signal(self, signal: dict[str, Any]) -> None:
        self._signals.append(dict(signal))

    def create_position(self, position: dict[str, Any], exit_rules: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = dict(position)
        payload["exit_rules"] = dict(exit_rules or {})
        payload["mode"] = "paper"
        payload["owner"] = "autoclose_engine"
        payload["paper_only"] = True
        payload["status"] = "open"
        payload["position_id"] = f"paper-opt-{len(self._positions)+1}"
        self._positions.append(payload)
        return payload

    def log_event(self, event: dict[str, Any]) -> None:
        self._events.append(dict(event))
        level = logging.INFO if event.get("event_type") != "options_pipeline_cycle_failed" else logging.ERROR
        logger.log(level, "options_pipeline_event %s", event)

    def has_recent_signal(self, symbol: str, strategy: str, expiry: str, within_minutes: int = 30) -> bool:
        now = datetime.now(timezone.utc)
        for sig in reversed(self._signals[-200:]):
            if (
                str(sig.get("symbol") or "").upper() == symbol.upper()
                and str(sig.get("strategy") or "").upper() == strategy.upper()
                and str(sig.get("expiry") or "") == expiry
            ):
                ts_raw = str(sig.get("timestamp") or "")
                try:
                    ts = datetime.fromisoformat(ts_raw.replace("Z", "+00:00"))
                    if ts.tzinfo is None:
                        ts = ts.replace(tzinfo=timezone.utc)
                    if (now - ts) <= timedelta(minutes=max(1, within_minutes)):
                        return True
                except Exception:
                    return True
        return False


def _build_limits_from_config(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "max_trades_per_cycle": int(config["max_trades_per_cycle"]),
        "dedupe_window_minutes": int(config["dedupe_window_minutes"]),
        "max_corr": float(config["max_corr"]),
    }


def run_options_pipeline_cycle(
    atlas: Any,
    *,
    pipeline_runner: Callable[..., dict[str, Any]] = run_options_trading_pipeline,
    config: dict[str, Any] | None = None,
    dedupe: _LocalSignalDedupe | None = None,
) -> dict[str, Any]:
    cfg = dict(config or build_options_pipeline_runtime_config())
    now_utc = datetime.now(timezone.utc)
    cycle_id = now_utc.replace(second=0, microsecond=0).isoformat()
    cycle_started = time.perf_counter()
    dedupe_impl = dedupe or _LocalSignalDedupe()
    wrapped = _AtlasDedupeWrapper(atlas, dedupe_impl)
    wrapped.set_cycle_anchor(now_utc)
    if not bool(cfg.get("paper_only", True)):
        atlas.log_event(
            {
                "event_type": "options_pipeline_cycle_failed",
                "timestamp": now_utc.isoformat(),
                "cycle_id": cycle_id,
                "paper_only": False,
                "error": "options_pipeline_runtime_requires_paper_only",
            }
        )
        return {
            "ok": False,
            "cycle_id": cycle_id,
            "error": "options_pipeline_runtime_requires_paper_only",
            "duration_ms": 0.0,
        }

    started_event = {
        "event_type": "options_pipeline_cycle_started",
        "timestamp": now_utc.isoformat(),
        "cycle_id": cycle_id,
        "paper_only": bool(cfg.get("paper_only", True)),
        "market_phase": _market_phase_et(now_utc),
        "limits": _build_limits_from_config(cfg),
    }
    atlas.log_event(started_event)
    try:
        result = pipeline_runner(now_utc, wrapped, limits=_build_limits_from_config(cfg))
        duration_ms = round((time.perf_counter() - cycle_started) * 1000.0, 3)
        finished_event = {
            "event_type": "options_pipeline_cycle_finished",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "cycle_id": cycle_id,
            "duration_ms": duration_ms,
            "symbols_evaluated": int(result.get("universe_size") or 0),
            "opportunities_generated": int(result.get("opportunities") or 0),
            "signals_emitted": int(result.get("emitted") or 0),
            "entries_blocked": int(result.get("blocked") or 0),
        }
        atlas.log_event(finished_event)
        return {"ok": True, **result, "cycle_id": cycle_id, "duration_ms": duration_ms}
    except Exception as exc:
        duration_ms = round((time.perf_counter() - cycle_started) * 1000.0, 3)
        atlas.log_event(
            {
                "event_type": "options_pipeline_cycle_failed",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "cycle_id": cycle_id,
                "duration_ms": duration_ms,
                "symbols_evaluated": 0,
                "opportunities_generated": 0,
                "signals_emitted": 0,
                "entries_blocked": 0,
                "error": str(exc),
            }
        )
        return {"ok": False, "cycle_id": cycle_id, "error": str(exc), "duration_ms": duration_ms}


class OptionsPipelineRuntimeScheduler:
    def __init__(
        self,
        *,
        atlas: Any,
        config: dict[str, Any] | None = None,
        pipeline_runner: Callable[..., dict[str, Any]] = run_options_trading_pipeline,
    ) -> None:
        self._atlas = atlas
        self._config = dict(config or build_options_pipeline_runtime_config())
        self._pipeline_runner = pipeline_runner
        self._dedupe = _LocalSignalDedupe()
        self._running = False

    async def run_forever(self) -> None:
        self._running = True
        while self._running:
            now = datetime.now(timezone.utc)
            phase = _market_phase_et(now)
            if phase in {"market", "premarket"}:
                await asyncio.to_thread(
                    run_options_pipeline_cycle,
                    self._atlas,
                    pipeline_runner=self._pipeline_runner,
                    config=self._config,
                    dedupe=self._dedupe,
                )
                sleep_for = (
                    int(self._config["market_interval_seconds"])
                    if phase == "market"
                    else int(self._config["premarket_interval_seconds"])
                )
            else:
                sleep_for = int(self._config["premarket_interval_seconds"])
            await asyncio.sleep(max(1, sleep_for))

    def stop(self) -> None:
        self._running = False


def build_default_options_pipeline_scheduler() -> OptionsPipelineRuntimeScheduler:
    universe_env = str(os.environ.get("QUANT_OPTIONS_PIPELINE_UNIVERSE", "")).strip()
    universe = [x.strip().upper() for x in universe_env.split(",") if x.strip()] if universe_env else None
    atlas = PaperAtlasOptionsAdapter(universe=universe)
    return OptionsPipelineRuntimeScheduler(
        atlas=atlas,
        config=build_options_pipeline_runtime_config(),
        pipeline_runner=run_options_trading_pipeline,
    )

