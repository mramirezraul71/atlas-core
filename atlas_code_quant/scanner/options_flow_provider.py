"""Hybrid options-flow provider for Atlas Elite.

This module upgrades the scanner from a purely intraday order-flow proxy
to a hybrid microstructure plus listed-options signal.

Persistence degrades gracefully:
- Preferred backend: TimescaleDB / PostgreSQL when DSN + psycopg are available.
- Fallback backend: local SQLite telemetry store.
"""
from __future__ import annotations

import json
import logging
import sqlite3
from dataclasses import dataclass
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any

from config.settings import settings
from execution.option_chain_cache import OptionChainCache


logger = logging.getLogger("atlas.scanner.options_flow")


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
        if out != out:
            return default
        return out
    except (TypeError, ValueError):
        return default


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pct_change(a: float, b: float) -> float:
    if not b:
        return 0.0
    return (a / b - 1.0) * 100.0


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


def _direction_from_score(score_pct: float) -> str:
    if score_pct >= 55.0:
        return "alcista"
    if score_pct <= 45.0:
        return "bajista"
    return "neutral"


def _contract_mid(contract: dict[str, Any]) -> float:
    bid = _safe_float(contract.get("bid"), 0.0)
    ask = _safe_float(contract.get("ask"), 0.0)
    if bid > 0 and ask > 0:
        return (bid + ask) / 2.0
    if ask > 0:
        return ask
    if bid > 0:
        return bid
    return _safe_float(contract.get("last"), 0.0)


def _extract_iv(contract: dict[str, Any]) -> float:
    greeks = contract.get("greeks") or {}
    for key in ("mid_iv", "smv_vol", "bid_iv", "ask_iv"):
        value = _safe_float(greeks.get(key), 0.0)
        if value > 0:
            return value
    for key in ("greeks_iv", "implied_volatility", "iv"):
        value = _safe_float(contract.get(key), 0.0)
        if value > 0:
            return value
    return 0.0


def _normalize_option_type(contract: dict[str, Any]) -> str:
    raw = str(contract.get("option_type") or contract.get("type") or contract.get("symbol") or "").lower()
    if "put" in raw or raw.endswith("p"):
        return "put"
    return "call"


def _spot_price_from_quote(quote: dict[str, Any], price_hint: float = 0.0) -> float:
    for key in ("last", "close", "bid", "ask", "prevclose"):
        value = _safe_float(quote.get(key), 0.0)
        if value > 0:
            return value
    return _safe_float(price_hint, 0.0)


@dataclass
class _TelemetryBackendStatus:
    requested_backend: str
    active_backend: str
    degraded: bool
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "requested_backend": self.requested_backend,
            "active_backend": self.active_backend,
            "degraded": self.degraded,
            "reason": self.reason,
        }


class MarketTelemetryStore:
    """Persist market telemetry with TimescaleDB to SQLite degradation."""

    def __init__(
        self,
        *,
        enabled: bool | None = None,
        backend: str | None = None,
        sqlite_path: Path | None = None,
        timescaledb_dsn: str | None = None,
        psycopg_module: Any | None = None,
    ) -> None:
        self.enabled = settings.market_telemetry_enabled if enabled is None else bool(enabled)
        self.requested_backend = str(backend or settings.market_telemetry_backend or "sqlite").lower()
        self.sqlite_path = Path(sqlite_path or settings.market_telemetry_sqlite_path)
        self.timescaledb_dsn = str(timescaledb_dsn or settings.market_telemetry_timescaledb_dsn or "").strip()
        self._psycopg = psycopg_module
        self._sqlite_schema_ready = False
        self._timescaledb_schema_ready = False
        self._status = self._resolve_backend_status()

    def _resolve_backend_status(self) -> _TelemetryBackendStatus:
        if not self.enabled:
            return _TelemetryBackendStatus(
                requested_backend=self.requested_backend,
                active_backend="disabled",
                degraded=False,
                reason="market_telemetry_disabled",
            )
        if self.requested_backend == "timescaledb":
            psycopg = self._psycopg
            if psycopg is None:
                try:
                    import psycopg as imported_psycopg  # type: ignore[import-not-found]

                    psycopg = imported_psycopg
                except Exception:
                    psycopg = None
            self._psycopg = psycopg
            if psycopg is not None and self.timescaledb_dsn:
                return _TelemetryBackendStatus(
                    requested_backend="timescaledb",
                    active_backend="timescaledb",
                    degraded=False,
                    reason="timescaledb_ready",
                )
            reason = "timescaledb_dsn_missing" if not self.timescaledb_dsn else "psycopg_unavailable"
            return _TelemetryBackendStatus(
                requested_backend="timescaledb",
                active_backend="sqlite",
                degraded=True,
                reason=reason,
            )
        return _TelemetryBackendStatus(
            requested_backend=self.requested_backend,
            active_backend="sqlite",
            degraded=False,
            reason="sqlite_ready",
        )

    def status(self) -> dict[str, Any]:
        payload = self._status.to_dict()
        payload["sqlite_path"] = str(self.sqlite_path)
        payload["timescaledb_configured"] = bool(self.timescaledb_dsn)
        return payload

    def record_order_flow_snapshot(
        self,
        *,
        symbol: str,
        scope: str,
        snapshot: dict[str, Any],
        recorded_at: datetime | None = None,
    ) -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "backend": "disabled", "reason": "market_telemetry_disabled"}

        ts = (recorded_at or _utcnow()).astimezone(timezone.utc)
        payload = {
            "ts_utc": ts.isoformat(),
            "symbol": symbol.upper(),
            "scope": scope,
            "mode": str(snapshot.get("mode") or "unknown"),
            "direction": str(snapshot.get("direction") or "neutral"),
            "score_pct": round(_safe_float(snapshot.get("score_pct"), 50.0), 4),
            "confidence_pct": round(_safe_float(snapshot.get("confidence_pct"), 0.0), 4),
            "available": 1 if bool(snapshot.get("available")) else 0,
            "payload_json": json.dumps(snapshot, ensure_ascii=True, sort_keys=True),
        }

        if self._status.active_backend == "timescaledb":
            try:
                self._write_timescaledb(payload)
                return {"ok": True, "backend": "timescaledb", "reason": self._status.reason}
            except Exception as exc:
                logger.warning("TimescaleDB telemetry degraded to SQLite: %s", exc)
                self._status = _TelemetryBackendStatus(
                    requested_backend=self.requested_backend,
                    active_backend="sqlite",
                    degraded=True,
                    reason=f"timescaledb_write_error:{exc}",
                )

        self._write_sqlite(payload)
        return {"ok": True, "backend": "sqlite", "reason": self._status.reason}

    def _write_sqlite(self, payload: dict[str, Any]) -> None:
        self.sqlite_path.parent.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(self.sqlite_path, timeout=10) as conn:
            if not self._sqlite_schema_ready:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS order_flow_snapshots (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        ts_utc TEXT NOT NULL,
                        symbol TEXT NOT NULL,
                        scope TEXT NOT NULL,
                        mode TEXT NOT NULL,
                        direction TEXT NOT NULL,
                        score_pct REAL NOT NULL,
                        confidence_pct REAL NOT NULL,
                        available INTEGER NOT NULL,
                        payload_json TEXT NOT NULL
                    )
                    """
                )
                conn.execute(
                    "CREATE INDEX IF NOT EXISTS idx_order_flow_snapshots_symbol_ts ON order_flow_snapshots(symbol, ts_utc DESC)"
                )
                self._sqlite_schema_ready = True
            conn.execute(
                """
                INSERT INTO order_flow_snapshots (
                    ts_utc, symbol, scope, mode, direction,
                    score_pct, confidence_pct, available, payload_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    payload["ts_utc"],
                    payload["symbol"],
                    payload["scope"],
                    payload["mode"],
                    payload["direction"],
                    payload["score_pct"],
                    payload["confidence_pct"],
                    payload["available"],
                    payload["payload_json"],
                ),
            )
            conn.commit()

    def _write_timescaledb(self, payload: dict[str, Any]) -> None:
        if self._psycopg is None:
            raise RuntimeError("psycopg_unavailable")
        with self._psycopg.connect(self.timescaledb_dsn) as conn:
            with conn.cursor() as cur:
                if not self._timescaledb_schema_ready:
                    cur.execute(
                        """
                        CREATE TABLE IF NOT EXISTS order_flow_snapshots (
                            ts_utc TIMESTAMPTZ NOT NULL,
                            symbol TEXT NOT NULL,
                            scope TEXT NOT NULL,
                            mode TEXT NOT NULL,
                            direction TEXT NOT NULL,
                            score_pct DOUBLE PRECISION NOT NULL,
                            confidence_pct DOUBLE PRECISION NOT NULL,
                            available BOOLEAN NOT NULL,
                            payload_json JSONB NOT NULL
                        )
                        """
                    )
                    cur.execute(
                        "CREATE INDEX IF NOT EXISTS idx_order_flow_snapshots_symbol_ts ON order_flow_snapshots(symbol, ts_utc DESC)"
                    )
                    try:
                        cur.execute("CREATE EXTENSION IF NOT EXISTS timescaledb")
                    except Exception:
                        pass
                    try:
                        cur.execute(
                            "SELECT create_hypertable('order_flow_snapshots', 'ts_utc', if_not_exists => TRUE)"
                        )
                    except Exception:
                        pass
                    self._timescaledb_schema_ready = True
                cur.execute(
                    """
                    INSERT INTO order_flow_snapshots (
                        ts_utc, symbol, scope, mode, direction,
                        score_pct, confidence_pct, available, payload_json
                    ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s::jsonb)
                    """,
                    (
                        payload["ts_utc"],
                        payload["symbol"],
                        payload["scope"],
                        payload["mode"],
                        payload["direction"],
                        payload["score_pct"],
                        payload["confidence_pct"],
                        bool(payload["available"]),
                        payload["payload_json"],
                    ),
                )
            conn.commit()


class _TradierChainCacheAdapter:
    """Compatibility shim for OptionChainCache and the current Tradier client."""

    def __init__(self, client: Any) -> None:
        self._client = client

    def get_option_chain(self, symbol: str, expiration: str, scope: str = "paper") -> list[dict[str, Any]]:
        del scope
        return list(self._client.chain(symbol, expiration))

    def get_option_expirations(self, symbol: str, scope: str = "paper") -> list[str]:
        del scope
        return list(self._client.expirations(symbol))


class OptionsFlowProvider:
    """Compute options-flow alternative data snapshot for the scanner."""

    def __init__(
        self,
        *,
        telemetry_store: MarketTelemetryStore | None = None,
        chain_cache: OptionChainCache | None = None,
    ) -> None:
        self._telemetry_store = telemetry_store or MarketTelemetryStore()
        self._chain_cache = chain_cache or OptionChainCache(ttl_sec=settings.scanner_options_flow_cache_ttl_sec)

    def telemetry_status(self) -> dict[str, Any]:
        return self._telemetry_store.status()

    def build_snapshot(
        self,
        *,
        symbol: str,
        client: Any,
        scope: str = "paper",
        price_hint: float = 0.0,
        quote: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        symbol = symbol.upper()
        snapshot = {
            "available": False,
            "scope": scope,
            "mode": "options_flow",
            "direction": "neutral",
            "score_pct": 50.0,
            "confidence_pct": 0.0,
            "put_call_volume_ratio": 1.0,
            "put_call_oi_ratio": 1.0,
            "iv_term_structure_slope_pct": 0.0,
            "atm_skew_pct": 0.0,
            "gamma_bias_pct": 0.0,
            "front_dte": None,
            "back_dte": None,
            "expirations_used": [],
            "contracts_evaluated": 0,
            "spot_price": round(_safe_float(price_hint, 0.0), 4),
            "telemetry": self._telemetry_store.status(),
            "reason": "options_flow_unavailable",
        }

        if "/" in symbol:
            snapshot["reason"] = "asset_without_listed_options"
            return snapshot

        adapter = _TradierChainCacheAdapter(client)
        try:
            live_quote = quote or client.quote(symbol)
        except Exception as exc:
            snapshot["reason"] = f"options_quote_error:{exc}"
            return snapshot

        spot_price = _spot_price_from_quote(live_quote, price_hint=price_hint)
        if spot_price <= 0:
            snapshot["reason"] = "options_spot_missing"
            return snapshot
        snapshot["spot_price"] = round(spot_price, 4)

        try:
            expirations = list(self._chain_cache.get_expirations(symbol, adapter, scope=scope))
        except Exception as exc:
            snapshot["reason"] = f"expirations_error:{exc}"
            return snapshot
        if not expirations:
            snapshot["reason"] = "expirations_empty"
            return snapshot

        today = _utcnow().date()
        future_expiry_rows: list[tuple[str, int]] = []
        expiry_rows: list[tuple[str, int]] = []
        for raw_exp in expirations:
            try:
                exp_date = date.fromisoformat(str(raw_exp))
            except Exception:
                continue
            dte = (exp_date - today).days
            if dte < 0:
                continue
            future_expiry_rows.append((str(raw_exp), dte))
            if dte < settings.scanner_options_flow_min_dte or dte > settings.scanner_options_flow_max_dte:
                continue
            expiry_rows.append((str(raw_exp), dte))
        if not expiry_rows:
            expiry_rows = list(future_expiry_rows)
        if not expiry_rows:
            snapshot["reason"] = "no_future_expirations"
            return snapshot

        expiry_rows.sort(key=lambda item: item[1])
        chosen = expiry_rows[: settings.scanner_options_flow_expirations]
        front_exp, front_dte = chosen[0]
        back_exp, back_dte = chosen[-1]

        try:
            front_chain = list(self._chain_cache.get_or_fetch(symbol, front_exp, adapter, scope=scope))
            back_chain = list(self._chain_cache.get_or_fetch(symbol, back_exp, adapter, scope=scope))
        except Exception as exc:
            snapshot["reason"] = f"chain_error:{exc}"
            return snapshot
        if not front_chain:
            snapshot["reason"] = "front_chain_empty"
            return snapshot
        if not back_chain:
            back_chain = list(front_chain)

        front_valid = [row for row in front_chain if _safe_float(row.get("strike"), 0.0) > 0]
        back_valid = [row for row in back_chain if _safe_float(row.get("strike"), 0.0) > 0]
        if len(front_valid) < 4:
            snapshot["reason"] = "insufficient_front_contracts"
            return snapshot

        calls = [row for row in front_valid if _normalize_option_type(row) == "call"]
        puts = [row for row in front_valid if _normalize_option_type(row) == "put"]
        if not calls or not puts:
            snapshot["reason"] = "missing_call_put_balance"
            return snapshot

        front_atm_call = min(calls, key=lambda row: abs(_safe_float(row.get("strike"), 0.0) - spot_price))
        front_atm_put = min(puts, key=lambda row: abs(_safe_float(row.get("strike"), 0.0) - spot_price))
        back_calls = [row for row in back_valid if _normalize_option_type(row) == "call"] or calls
        back_puts = [row for row in back_valid if _normalize_option_type(row) == "put"] or puts
        back_atm_call = min(back_calls, key=lambda row: abs(_safe_float(row.get("strike"), 0.0) - spot_price))
        back_atm_put = min(back_puts, key=lambda row: abs(_safe_float(row.get("strike"), 0.0) - spot_price))

        call_volume = sum(_safe_float(row.get("volume"), 0.0) for row in calls)
        put_volume = sum(_safe_float(row.get("volume"), 0.0) for row in puts)
        call_oi = sum(_safe_float(row.get("open_interest"), _safe_float(row.get("openInterest"), 0.0)) for row in calls)
        put_oi = sum(_safe_float(row.get("open_interest"), _safe_float(row.get("openInterest"), 0.0)) for row in puts)
        put_call_volume_ratio = put_volume / max(call_volume, 1.0)
        put_call_oi_ratio = put_oi / max(call_oi, 1.0)

        front_call_iv = _extract_iv(front_atm_call)
        front_put_iv = _extract_iv(front_atm_put)
        back_call_iv = _extract_iv(back_atm_call)
        back_put_iv = _extract_iv(back_atm_put)
        front_mid_iv = max((front_call_iv + front_put_iv) / 2.0, 1e-6)
        back_mid_iv = max((back_call_iv + back_put_iv) / 2.0, 1e-6)
        iv_term_structure_slope_pct = _pct_change(back_mid_iv, front_mid_iv)
        atm_skew_pct = ((front_put_iv - front_call_iv) / front_mid_iv) * 100.0 if front_mid_iv > 0 else 0.0

        gamma_signed = 0.0
        gamma_abs = 0.0
        notional_premium = 0.0
        for row in front_valid:
            gamma = _safe_float((row.get("greeks") or {}).get("gamma"), _safe_float(row.get("gamma"), 0.0))
            oi = _safe_float(row.get("open_interest"), _safe_float(row.get("openInterest"), 0.0))
            signed = gamma * max(oi, 0.0) * (1.0 if _normalize_option_type(row) == "call" else -1.0)
            gamma_signed += signed
            gamma_abs += abs(gamma * max(oi, 0.0))
            notional_premium += _contract_mid(row) * max(oi, 0.0)
        gamma_bias_pct = (gamma_signed / gamma_abs * 100.0) if gamma_abs > 0 else 0.0

        ratio_bias = _clip((1.0 - put_call_volume_ratio) * 28.0, -18.0, 18.0)
        oi_bias = _clip((1.0 - put_call_oi_ratio) * 22.0, -14.0, 14.0)
        gamma_bias = _clip(gamma_bias_pct * 0.18, -14.0, 14.0)
        term_bias = _clip(iv_term_structure_slope_pct * 0.75, -10.0, 10.0)
        skew_bias = _clip(-atm_skew_pct * 0.35, -10.0, 10.0)
        liquidity_bias = _clip(((call_volume + put_volume) / max(len(front_valid), 1)) / 25.0, 0.0, 6.0)
        raw_score = 50.0 + ratio_bias + oi_bias + gamma_bias + term_bias + skew_bias + liquidity_bias
        score_pct = _clip(raw_score, 0.0, 100.0)
        contracts_evaluated = len(front_valid) + len(back_valid)
        data_richness_pct = _clip((contracts_evaluated / 80.0) * 100.0, 0.0, 100.0)
        confidence_pct = _clip(abs(score_pct - 50.0) * 1.7 + data_richness_pct * 0.25, 0.0, 100.0)

        snapshot.update(
            {
                "available": True,
                "direction": _direction_from_score(score_pct),
                "score_pct": round(score_pct, 2),
                "confidence_pct": round(confidence_pct, 2),
                "put_call_volume_ratio": round(put_call_volume_ratio, 4),
                "put_call_oi_ratio": round(put_call_oi_ratio, 4),
                "iv_term_structure_slope_pct": round(iv_term_structure_slope_pct, 3),
                "atm_skew_pct": round(atm_skew_pct, 3),
                "gamma_bias_pct": round(gamma_bias_pct, 3),
                "front_dte": front_dte,
                "back_dte": back_dte,
                "expirations_used": [front_exp] if front_exp == back_exp else [front_exp, back_exp],
                "contracts_evaluated": contracts_evaluated,
                "front_atm_call_iv": round(front_call_iv, 4),
                "front_atm_put_iv": round(front_put_iv, 4),
                "back_atm_call_iv": round(back_call_iv, 4),
                "back_atm_put_iv": round(back_put_iv, 4),
                "notional_premium_proxy": round(notional_premium, 2),
                "reason": "options_flow_ok",
            }
        )
        snapshot["telemetry_write"] = self._telemetry_store.record_order_flow_snapshot(
            symbol=symbol,
            scope=scope,
            snapshot=snapshot,
        )
        snapshot["telemetry"] = self._telemetry_store.status()
        return snapshot
