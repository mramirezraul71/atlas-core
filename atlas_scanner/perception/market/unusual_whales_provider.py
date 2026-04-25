from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import logging
import os
import time
from typing import Any, Mapping

import requests

from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from .flow_normalizer import RawFlowEvent

logger = logging.getLogger("atlas_scanner.flow.unusual_whales")

_BASE_URL = "https://api.unusualwhales.com"


@dataclass(frozen=True)
class UnusualWhalesProviderConfig:
    api_key: str
    base_url: str = _BASE_URL
    timeout_sec: float = 8.0
    max_events: int = 200
    min_premium: float = 10_000.0
    client_api_id: str | None = None


@dataclass
class UnusualWhalesFlowProvider:
    config: UnusualWhalesProviderConfig
    last_diagnostics: dict[str, Any] = field(default_factory=dict)

    def fetch_events(self, *, symbol: str, since: datetime, until: datetime) -> tuple[RawFlowEvent, ...]:
        _ = since, until
        breaker = resolve_provider_circuit_breaker("flow:unusual_whales")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "error": "circuit_open",
                "provider": "unusual_whales",
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return ()
        url = f"{self.config.base_url.rstrip('/')}/api/option-trades/flow-alerts"
        headers = {
            "Authorization": f"Bearer {self.config.api_key}",
            "Accept": "application/json",
        }
        if self.config.client_api_id:
            headers["UW-CLIENT-API-ID"] = self.config.client_api_id
        params = {
            "ticker_symbol": symbol.upper(),
            "min_premium": self.config.min_premium,
            "limit": min(max(1, int(self.config.max_events)), 200),
        }
        t0 = time.perf_counter()
        try:
            response = requests.get(url, headers=headers, params=params, timeout=self.config.timeout_sec)
            latency_ms = int((time.perf_counter() - t0) * 1000)
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "error": str(exc),
                "provider": "unusual_whales",
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            logger.warning("unusual_whales request failed symbol=%s error=%s", symbol, exc)
            return ()
        headers_diag = _extract_rate_limit_headers(dict(response.headers))
        if response.status_code >= 400:
            breaker.record_failure(f"http_{response.status_code}")
            self.last_diagnostics = {
                "status": "error",
                "http_status": response.status_code,
                "provider": "unusual_whales",
                "latency_ms": latency_ms,
                "rate_limit": headers_diag,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            logger.warning(
                "unusual_whales bad response symbol=%s status=%s latency_ms=%s",
                symbol,
                response.status_code,
                latency_ms,
            )
            return ()
        payload = response.json() if response.content else {}
        rows = payload.get("data")
        if not isinstance(rows, list):
            rows = []
        events = tuple(
            event
            for row in rows
            if isinstance(row, Mapping)
            for event in (_map_alert_row(symbol=symbol.upper(), row=row),)
            if event is not None
        )
        self.last_diagnostics = {
            "status": "ok" if events else "empty",
            "provider": "unusual_whales",
            "latency_ms": latency_ms,
            "rows": len(rows),
            "events": len(events),
            "rate_limit": headers_diag,
            "circuit_breaker": _cb_dict(breaker.snapshot()),
        }
        if events:
            breaker.record_success()
        return events


def config_from_env() -> UnusualWhalesProviderConfig | None:
    token = os.getenv("ATLAS_UNUSUAL_WHALES_API_KEY", "").strip()
    if not token:
        return None
    timeout_sec = float(os.getenv("ATLAS_UNUSUAL_WHALES_TIMEOUT_SEC", "8.0"))
    max_events = int(os.getenv("ATLAS_UNUSUAL_WHALES_MAX_EVENTS", "120"))
    min_premium = float(os.getenv("ATLAS_UNUSUAL_WHALES_MIN_PREMIUM", "10000"))
    client_api_id = os.getenv("ATLAS_UNUSUAL_WHALES_CLIENT_API_ID", "").strip() or None
    base_url = os.getenv("ATLAS_UNUSUAL_WHALES_BASE_URL", _BASE_URL).strip() or _BASE_URL
    return UnusualWhalesProviderConfig(
        api_key=token,
        base_url=base_url,
        timeout_sec=timeout_sec,
        max_events=max_events,
        min_premium=min_premium,
        client_api_id=client_api_id,
    )


def _parse_datetime(value: object) -> datetime:
    if isinstance(value, str):
        iso = value.replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(iso)
        except ValueError:
            pass
    return datetime.now(timezone.utc)


def _as_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.replace(",", ""))
        except ValueError:
            return None
    return None


def _map_alert_row(*, symbol: str, row: Mapping[str, Any]) -> RawFlowEvent | None:
    event_ts = _parse_datetime(row.get("created_at"))
    option_type = str(row.get("type", "")).strip().lower()
    flow_type: str = "call" if option_type == "call" else "put" if option_type == "put" else "equity"
    ask_premium = _as_float(row.get("total_ask_side_prem")) or 0.0
    bid_premium = _as_float(row.get("total_bid_side_prem")) or 0.0
    total_premium = _as_float(row.get("total_premium")) or (ask_premium + bid_premium)
    size = _as_float(row.get("total_size")) or 0.0
    strike = _as_float(row.get("strike"))
    expiry_str = row.get("expiry")
    dte = None
    if isinstance(expiry_str, str):
        try:
            expiry = datetime.fromisoformat(expiry_str).date()
            dte = max(0, (expiry - event_ts.date()).days)
        except ValueError:
            dte = None
    aggression = "aggressive" if ask_premium >= bid_premium else "passive"
    side = "buy" if ask_premium >= bid_premium else "sell"
    kind = "sweep" if bool(row.get("has_sweep")) else "block" if bool(row.get("has_floor")) else "regular"
    if flow_type == "equity":
        return None
    return RawFlowEvent(
        symbol=symbol,
        event_ts=event_ts,
        side=side,  # type: ignore[arg-type]
        size=size,
        strike=strike,
        dte=dte,
        premium=total_premium,
        aggression=aggression,  # type: ignore[arg-type]
        type=flow_type,  # type: ignore[arg-type]
        event_kind=kind,  # type: ignore[arg-type]
        meta={
            "option_chain": row.get("option_chain"),
            "volume": row.get("volume"),
            "open_interest": row.get("open_interest"),
            "provider": "unusual_whales",
        },
    )


def _extract_rate_limit_headers(headers: Mapping[str, str]) -> dict[str, str]:
    keys = (
        "x-uw-daily-req-count",
        "x-uw-token-req-limit",
        "x-uw-minute-req-counter",
        "x-uw-req-per-minute-remaining",
        "x-uw-req-per-minute-reset",
    )
    normalized = {k.lower(): v for k, v in headers.items()}
    return {key: normalized.get(key, "") for key in keys if key in normalized}


def _cb_dict(snapshot: Any) -> dict[str, Any]:
    return {
        "state": getattr(snapshot, "state", "unknown"),
        "consecutive_failures": getattr(snapshot, "consecutive_failures", 0),
        "failure_threshold": getattr(snapshot, "failure_threshold", 0),
        "cooldown_sec": getattr(snapshot, "cooldown_sec", 0),
        "last_error": getattr(snapshot, "last_error", None),
    }
