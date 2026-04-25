from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import os
from typing import Any
from typing import Protocol

import requests

from atlas_scanner.contracts import PoliticalTradingSnapshot
from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from atlas_scanner.perception.political.pipeline import build_political_trading_context


class PoliticalTradingProvider(Protocol):
    def fetch(self, *, scope: str, as_of: datetime) -> PoliticalTradingSnapshot:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


@dataclass
class StubPoliticalTradingProvider:
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "political_stub"}

    def fetch(self, *, scope: str, as_of: datetime) -> PoliticalTradingSnapshot:
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "political_stub",
            "provider_ready": False,
            "fallback_active": True,
        }
        return build_political_trading_context(
            scope=scope,
            as_of=as_of,
            source="political_stub",
            provider_payload={
                "net_political_flow": 0.0,
                "related_tickers": [scope.upper()],
                "disclosure_lag_days": 30,
                "transaction_count": 0,
                "buy_count": 0,
                "sell_count": 0,
                "notable_entities": (),
                "signal_strength": "weak",
                "freshness_sec": 21 * 24 * 3600,
                "delay_sec": 30 * 24 * 3600,
                "confidence": 0.15,
                "provider_ready": False,
            },
        )


@dataclass(frozen=True)
class FinnhubPoliticalProviderConfig:
    api_key: str
    base_url: str = "https://finnhub.io/api/v1/stock/congressional-trading"
    timeout_sec: float = 6.0
    lookback_days: int = 120
    max_rows: int = 200


@dataclass
class FinnhubPoliticalTradingProvider:
    config: FinnhubPoliticalProviderConfig
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "political_finnhub"}

    def fetch(self, *, scope: str, as_of: datetime) -> PoliticalTradingSnapshot:
        breaker = resolve_provider_circuit_breaker("political:finnhub")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "political_finnhub",
                "error": "circuit_open",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(scope=scope, as_of=as_of, reason="circuit_open")

        params = {
            "symbol": scope.upper(),
            "token": self.config.api_key,
        }
        try:
            response = requests.get(self.config.base_url, params=params, timeout=self.config.timeout_sec)
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "political_finnhub",
                "error": str(exc),
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(scope=scope, as_of=as_of, reason="request_exception")

        if response.status_code >= 400:
            breaker.record_failure(f"http_{response.status_code}")
            self.last_diagnostics = {
                "status": "error",
                "provider": "political_finnhub",
                "error": f"http_{response.status_code}",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(scope=scope, as_of=as_of, reason=f"http_{response.status_code}")

        payload = response.json()
        rows = _extract_rows(payload)
        cutoff = _to_utc(as_of) - timedelta(days=max(1, self.config.lookback_days))
        selected = [row for row in rows[: max(1, self.config.max_rows)] if _is_recent(row=row, cutoff=cutoff)]
        agg = _aggregate_political_rows(rows=selected, as_of=as_of, fallback_scope=scope.upper())

        provider_ready = agg["transaction_count"] > 0 and agg["net_political_flow"] is not None
        if provider_ready:
            breaker.record_success()
        else:
            breaker.record_failure("missing_critical_fields")

        self.last_diagnostics = {
            "status": "ok" if provider_ready else "degraded",
            "provider": "political_finnhub",
            "provider_ready": provider_ready,
            "fallback_active": not provider_ready,
            "rows": len(rows),
            "selected_rows": len(selected),
            "lookback_days": self.config.lookback_days,
            "latency_ms": int((agg.get("latency_ms") or 0)),
            "circuit_breaker": _cb_dict(breaker.snapshot()),
        }
        if not provider_ready:
            return _fallback_snapshot(scope=scope, as_of=as_of, reason="missing_critical_fields")

        return build_political_trading_context(
            scope=scope,
            as_of=as_of,
            source="political_finnhub",
            provider_payload={
                "net_political_flow": agg["net_political_flow"],
                "related_tickers": agg["related_tickers"],
                "disclosure_lag_days": agg["disclosure_lag_days"],
                "freshness_sec": agg["freshness_sec"],
                "delay_sec": agg["delay_sec"],
                "confidence": agg["confidence"],
                "provider_ready": True,
                "transaction_count": agg["transaction_count"],
                "buy_count": agg["buy_count"],
                "sell_count": agg["sell_count"],
                "buy_sell_balance": agg["buy_sell_balance"],
                "notable_entities": agg["notable_entities"],
                "signal_strength": agg["signal_strength"],
            },
        )


def resolve_political_provider() -> PoliticalTradingProvider:
    provider_name = os.getenv("ATLAS_POLITICAL_PROVIDER", "stub").strip().lower()
    if provider_name in {"finnhub"}:
        key = os.getenv("ATLAS_FINNHUB_API_KEY", "").strip()
        if key:
            timeout = float(os.getenv("ATLAS_POLITICAL_TIMEOUT_SEC", "6.0"))
            lookback_days = int(os.getenv("ATLAS_POLITICAL_LOOKBACK_DAYS", "120"))
            max_rows = int(os.getenv("ATLAS_POLITICAL_MAX_ROWS", "200"))
            base_url = os.getenv(
                "ATLAS_POLITICAL_BASE_URL",
                "https://finnhub.io/api/v1/stock/congressional-trading",
            )
            return FinnhubPoliticalTradingProvider(
                config=FinnhubPoliticalProviderConfig(
                    api_key=key,
                    base_url=base_url,
                    timeout_sec=timeout,
                    lookback_days=lookback_days,
                    max_rows=max_rows,
                )
            )
    return StubPoliticalTradingProvider()


def _fallback_snapshot(*, scope: str, as_of: datetime, reason: str) -> PoliticalTradingSnapshot:
    return build_political_trading_context(
        scope=scope,
        as_of=as_of,
        source="political_fallback_stub",
        provider_payload={
            "net_political_flow": 0.0,
            "related_tickers": [scope.upper()],
            "disclosure_lag_days": 30,
            "transaction_count": 0,
            "buy_count": 0,
            "sell_count": 0,
            "buy_sell_balance": 0.0,
            "notable_entities": (),
            "signal_strength": "weak",
            "freshness_sec": 21 * 24 * 3600,
            "delay_sec": 30 * 24 * 3600,
            "confidence": 0.2,
            "provider_ready": False,
            "fallback_reason": reason,
        },
    )


def _extract_rows(payload: Any) -> list[dict[str, Any]]:
    if isinstance(payload, list):
        return [row for row in payload if isinstance(row, dict)]
    if isinstance(payload, dict):
        for key in ("data", "results", "transactions"):
            value = payload.get(key)
            if isinstance(value, list):
                return [row for row in value if isinstance(row, dict)]
    return []


def _aggregate_political_rows(*, rows: list[dict[str, Any]], as_of: datetime, fallback_scope: str) -> dict[str, Any]:
    if not rows:
        return {
            "net_political_flow": None,
            "related_tickers": (fallback_scope,),
            "disclosure_lag_days": None,
            "freshness_sec": 21 * 24 * 3600,
            "delay_sec": None,
            "confidence": 0.25,
            "transaction_count": 0,
            "buy_count": 0,
            "sell_count": 0,
            "buy_sell_balance": 0.0,
            "notable_entities": (),
            "signal_strength": "weak",
            "latency_ms": 0,
        }

    buy_count = 0
    sell_count = 0
    amount_buy = 0.0
    amount_sell = 0.0
    entities: list[str] = []
    tickers: list[str] = []
    lag_days: list[int] = []
    for row in rows:
        side = str(row.get("type") or row.get("transactionType") or row.get("transaction") or "").lower()
        amount = _as_float(row.get("amount") or row.get("transactionAmount") or row.get("value") or 1.0) or 1.0
        ticker = str(row.get("symbol") or row.get("ticker") or fallback_scope).upper()
        actor = str(row.get("name") or row.get("representative") or row.get("politician") or "unknown").strip()
        if ticker:
            tickers.append(ticker)
        if actor:
            entities.append(actor)
        if "buy" in side or side in {"a", "acquisition", "purchase"}:
            buy_count += 1
            amount_buy += amount
        elif "sell" in side or side in {"d", "disposition"}:
            sell_count += 1
            amount_sell += amount
        filed_at = _parse_datetime(row.get("disclosureDate") or row.get("disclosure_date"))
        transacted_at = _parse_datetime(row.get("transactionDate") or row.get("transaction_date"))
        if filed_at is not None and transacted_at is not None:
            lag_days.append(max(0, int((_to_utc(filed_at) - _to_utc(transacted_at)).total_seconds() // 86400)))

    tx_count = buy_count + sell_count
    if tx_count <= 0:
        return {
            "net_political_flow": None,
            "related_tickers": tuple(sorted(set(tickers))) or (fallback_scope,),
            "disclosure_lag_days": int(sum(lag_days) / len(lag_days)) if lag_days else None,
            "freshness_sec": 21 * 24 * 3600,
            "delay_sec": (int(sum(lag_days) / len(lag_days)) * 86400) if lag_days else None,
            "confidence": 0.3,
            "transaction_count": 0,
            "buy_count": buy_count,
            "sell_count": sell_count,
            "buy_sell_balance": 0.0,
            "notable_entities": tuple(sorted(set(entities))[:5]),
            "signal_strength": "weak",
            "latency_ms": 0,
        }

    balance = (amount_buy - amount_sell) / max(1.0, amount_buy + amount_sell)
    avg_lag = int(sum(lag_days) / len(lag_days)) if lag_days else 30
    confidence = 0.45
    if tx_count >= 8:
        confidence = 0.65
    if tx_count >= 16:
        confidence = 0.78
    if avg_lag > 45:
        confidence = max(0.3, confidence - 0.15)
    strength = "weak"
    abs_balance = abs(balance)
    if abs_balance >= 0.25 and tx_count >= 4:
        strength = "medium"
    if abs_balance >= 0.45 and tx_count >= 8:
        strength = "strong"
    return {
        "net_political_flow": balance,
        "related_tickers": tuple(sorted(set(tickers))) or (fallback_scope,),
        "disclosure_lag_days": avg_lag,
        "freshness_sec": 14 * 24 * 3600,
        "delay_sec": avg_lag * 86400,
        "confidence": confidence,
        "transaction_count": tx_count,
        "buy_count": buy_count,
        "sell_count": sell_count,
        "buy_sell_balance": balance,
        "notable_entities": tuple(sorted(set(entities))[:5]),
        "signal_strength": strength,
        "latency_ms": 0,
    }


def _is_recent(*, row: dict[str, Any], cutoff: datetime) -> bool:
    filed_at = _parse_datetime(row.get("disclosureDate") or row.get("disclosure_date") or row.get("disclosure"))
    tx_at = _parse_datetime(row.get("transactionDate") or row.get("transaction_date"))
    candidate = filed_at or tx_at
    if candidate is None:
        return False
    return _to_utc(candidate) >= cutoff


def _parse_datetime(value: object) -> datetime | None:
    if isinstance(value, str):
        iso = value.replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(iso)
        except ValueError:
            return None
    return None


def _as_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.replace(",", ""))
        except ValueError:
            return None
    return None


def _to_utc(value: datetime) -> datetime:
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def _cb_dict(snapshot: Any) -> dict[str, Any]:
    return {
        "state": getattr(snapshot, "state", "unknown"),
        "consecutive_failures": getattr(snapshot, "consecutive_failures", 0),
        "failure_threshold": getattr(snapshot, "failure_threshold", 0),
        "cooldown_sec": getattr(snapshot, "cooldown_sec", 0),
        "last_error": getattr(snapshot, "last_error", None),
    }
