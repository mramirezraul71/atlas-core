from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import os
from typing import Any
from typing import Protocol

import requests

from atlas_scanner.contracts import InsiderTradingSnapshot, InstitutionalOwnershipSnapshot
from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from atlas_scanner.perception.institutional.pipeline import build_insider_trading, build_institutional_ownership


class InstitutionalOwnershipProvider(Protocol):
    def fetch(self, *, symbol: str, as_of: datetime) -> InstitutionalOwnershipSnapshot:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


class InsiderTradingProvider(Protocol):
    def fetch(self, *, symbol: str, as_of: datetime) -> InsiderTradingSnapshot:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


@dataclass
class StubInstitutionalOwnershipProvider:
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "ownership_stub"}

    def fetch(self, *, symbol: str, as_of: datetime) -> InstitutionalOwnershipSnapshot:
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "ownership_stub",
            "provider_ready": False,
            "fallback_active": True,
        }
        return build_institutional_ownership(
            symbol=symbol,
            as_of=as_of,
            source="ownership_stub",
            provider_payload={
                "ownership_pct": 0.65,
                "ownership_delta_pct": 0.0,
                "concentration_score": 0.5,
                "sponsorship_score": 0.45,
                "holder_change_balance": 0.0,
                "ownership_signal": "neutral",
                "top_holders": (),
                "delay_days": 45,
                "freshness_sec": 120 * 24 * 3600,
                "confidence": 0.2,
                "provider_ready": False,
            },
        )


@dataclass
class StubInsiderTradingProvider:
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "insider_stub"}

    def fetch(self, *, symbol: str, as_of: datetime) -> InsiderTradingSnapshot:
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "insider_stub",
            "provider_ready": False,
            "fallback_active": True,
        }
        return build_insider_trading(
            symbol=symbol,
            as_of=as_of,
            source="insider_stub",
            provider_payload={
                "buy_sell_ratio": 1.0,
                "net_insider_value": 0.0,
                "transaction_count": 0,
                "net_signal": "neutral",
                "confidence": 0.2,
                "provider_ready": False,
            },
        )


@dataclass(frozen=True)
class FmpInsiderProviderConfig:
    api_key: str
    base_url: str = "https://financialmodelingprep.com/api/v4/insider-trading"
    timeout_sec: float = 6.0
    lookback_days: int = 45
    max_rows: int = 100


@dataclass
class FmpInsiderTradingProvider:
    config: FmpInsiderProviderConfig
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "insider_fmp"}

    def fetch(self, *, symbol: str, as_of: datetime) -> InsiderTradingSnapshot:
        breaker = resolve_provider_circuit_breaker("insider:fmp")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "insider_fmp",
                "error": "circuit_open",
                "fallback_active": True,
                "provider_ready": False,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol=symbol, as_of=as_of, reason="circuit_open")

        params = {
            "symbol": symbol.upper(),
            "page": 0,
            "apikey": self.config.api_key,
        }
        try:
            response = requests.get(self.config.base_url, params=params, timeout=self.config.timeout_sec)
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "insider_fmp",
                "error": str(exc),
                "fallback_active": True,
                "provider_ready": False,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol=symbol, as_of=as_of, reason="request_exception")

        if response.status_code >= 400:
            breaker.record_failure(f"http_{response.status_code}")
            self.last_diagnostics = {
                "status": "error",
                "provider": "insider_fmp",
                "error": f"http_{response.status_code}",
                "fallback_active": True,
                "provider_ready": False,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol=symbol, as_of=as_of, reason=f"http_{response.status_code}")

        rows = response.json()
        if not isinstance(rows, list):
            breaker.record_failure("invalid_payload")
            self.last_diagnostics = {
                "status": "error",
                "provider": "insider_fmp",
                "error": "invalid_payload",
                "fallback_active": True,
                "provider_ready": False,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol=symbol, as_of=as_of, reason="invalid_payload")

        cutoff = _to_utc(as_of) - timedelta(days=max(1, self.config.lookback_days))
        filtered = [row for row in rows if isinstance(row, dict) and _is_recent(row=row, cutoff=cutoff)]
        selected = filtered[: max(1, self.config.max_rows)]
        agg = _aggregate_insider_rows(selected)

        provider_ready = agg["transaction_count"] > 0 and agg["buy_sell_ratio"] is not None
        if provider_ready:
            breaker.record_success()
        else:
            breaker.record_failure("missing_critical_fields")
        fallback_active = not provider_ready
        self.last_diagnostics = {
            "status": "ok" if provider_ready else "degraded",
            "provider": "insider_fmp",
            "provider_ready": provider_ready,
            "fallback_active": fallback_active,
            "rows": len(rows),
            "selected_rows": len(selected),
            "lookback_days": self.config.lookback_days,
            "circuit_breaker": _cb_dict(breaker.snapshot()),
        }
        if not provider_ready:
            return _fallback_snapshot(symbol=symbol, as_of=as_of, reason="missing_critical_fields")

        return build_insider_trading(
            symbol=symbol,
            as_of=as_of,
            source="insider_fmp",
            provider_payload={
                "buy_sell_ratio": agg["buy_sell_ratio"],
                "net_insider_value": agg["net_insider_value"],
                "notable_buyers": agg["notable_buyers"],
                "notable_sellers": agg["notable_sellers"],
                "freshness_sec": agg["freshness_sec"],
                "delay_sec": agg["delay_sec"],
                "confidence": agg["confidence"],
                "provider_ready": True,
                "transaction_count": agg["transaction_count"],
                "net_signal": agg["net_signal"],
            },
        )


@dataclass(frozen=True)
class FmpOwnershipProviderConfig:
    api_key: str
    base_url: str = "https://financialmodelingprep.com/stable/institutional-ownership/symbol-positions-summary"
    timeout_sec: float = 6.0
    lookback_quarters: int = 4
    max_rows: int = 80


@dataclass
class FmpInstitutionalOwnershipProvider:
    config: FmpOwnershipProviderConfig
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "ownership_fmp"}

    def fetch(self, *, symbol: str, as_of: datetime) -> InstitutionalOwnershipSnapshot:
        breaker = resolve_provider_circuit_breaker("ownership:fmp")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "ownership_fmp",
                "error": "circuit_open",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_ownership_snapshot(symbol=symbol, as_of=as_of, reason="circuit_open")

        params = {
            "symbol": symbol.upper(),
            "apikey": self.config.api_key,
            "page": 0,
            "limit": max(1, self.config.max_rows),
        }
        try:
            response = requests.get(self.config.base_url, params=params, timeout=self.config.timeout_sec)
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "ownership_fmp",
                "error": str(exc),
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_ownership_snapshot(symbol=symbol, as_of=as_of, reason="request_exception")
        if response.status_code >= 400:
            breaker.record_failure(f"http_{response.status_code}")
            self.last_diagnostics = {
                "status": "error",
                "provider": "ownership_fmp",
                "error": f"http_{response.status_code}",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_ownership_snapshot(symbol=symbol, as_of=as_of, reason=f"http_{response.status_code}")
        payload = response.json()
        if not isinstance(payload, list):
            breaker.record_failure("invalid_payload")
            self.last_diagnostics = {
                "status": "error",
                "provider": "ownership_fmp",
                "error": "invalid_payload",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_ownership_snapshot(symbol=symbol, as_of=as_of, reason="invalid_payload")

        parsed = _aggregate_ownership_rows(rows=[row for row in payload if isinstance(row, dict)], as_of=as_of)
        provider_ready = parsed["ownership_pct"] is not None and parsed["concentration_score"] is not None
        if provider_ready:
            breaker.record_success()
        else:
            breaker.record_failure("missing_critical_fields")
        self.last_diagnostics = {
            "status": "ok" if provider_ready else "degraded",
            "provider": "ownership_fmp",
            "provider_ready": provider_ready,
            "fallback_active": not provider_ready,
            "rows": len(payload),
            "lookback_quarters": self.config.lookback_quarters,
            "circuit_breaker": _cb_dict(breaker.snapshot()),
        }
        if not provider_ready:
            return _fallback_ownership_snapshot(symbol=symbol, as_of=as_of, reason="missing_critical_fields")

        return build_institutional_ownership(
            symbol=symbol,
            as_of=as_of,
            source="ownership_fmp",
            provider_payload={
                "ownership_pct": parsed["ownership_pct"],
                "ownership_delta_pct": parsed["ownership_delta_pct"],
                "concentration_score": parsed["concentration_score"],
                "freshness_sec": 120 * 24 * 3600,
                "delay_sec": parsed["delay_sec"],
                "confidence": parsed["confidence"],
                "provider_ready": True,
                "top_holders": parsed["top_holders"],
                "holder_change_balance": parsed["holder_change_balance"],
                "sponsorship_score": parsed["sponsorship_score"],
                "ownership_signal": parsed["ownership_signal"],
                "delay_days": parsed["delay_days"],
                "filing_lag_note": "13F filings may lag up to 45 days post-quarter",
            },
        )


def resolve_institutional_ownership_provider() -> InstitutionalOwnershipProvider:
    provider_name = os.getenv("ATLAS_INSTITUTIONAL_PROVIDER", "stub").strip().lower()
    if provider_name in {"fmp", "financialmodelingprep"}:
        key = os.getenv("ATLAS_FMP_API_KEY", "").strip()
        if key:
            timeout = float(os.getenv("ATLAS_INSTITUTIONAL_TIMEOUT_SEC", "6.0"))
            lookback_quarters = int(os.getenv("ATLAS_INSTITUTIONAL_LOOKBACK_QUARTERS", "4"))
            max_rows = int(os.getenv("ATLAS_INSTITUTIONAL_MAX_ROWS", "80"))
            base_url = os.getenv(
                "ATLAS_INSTITUTIONAL_BASE_URL",
                "https://financialmodelingprep.com/stable/institutional-ownership/symbol-positions-summary",
            )
            return FmpInstitutionalOwnershipProvider(
                config=FmpOwnershipProviderConfig(
                    api_key=key,
                    timeout_sec=timeout,
                    lookback_quarters=lookback_quarters,
                    max_rows=max_rows,
                    base_url=base_url,
                )
            )
    return StubInstitutionalOwnershipProvider()


def resolve_insider_provider() -> InsiderTradingProvider:
    provider_name = os.getenv("ATLAS_INSIDER_PROVIDER", "stub").strip().lower()
    if provider_name in {"fmp", "financialmodelingprep"}:
        key = os.getenv("ATLAS_FMP_API_KEY", "").strip()
        if key:
            timeout = float(os.getenv("ATLAS_INSIDER_TIMEOUT_SEC", "6.0"))
            lookback_days = int(os.getenv("ATLAS_INSIDER_LOOKBACK_DAYS", "45"))
            max_rows = int(os.getenv("ATLAS_INSIDER_MAX_ROWS", "100"))
            base_url = os.getenv("ATLAS_INSIDER_BASE_URL", "https://financialmodelingprep.com/api/v4/insider-trading")
            return FmpInsiderTradingProvider(
                config=FmpInsiderProviderConfig(
                    api_key=key,
                    timeout_sec=timeout,
                    lookback_days=lookback_days,
                    max_rows=max_rows,
                    base_url=base_url,
                )
            )
    return StubInsiderTradingProvider()


def _fallback_snapshot(*, symbol: str, as_of: datetime, reason: str) -> InsiderTradingSnapshot:
    return build_insider_trading(
        symbol=symbol,
        as_of=as_of,
        source="insider_fallback_stub",
        provider_payload={
            "buy_sell_ratio": 1.0,
            "net_insider_value": 0.0,
            "confidence": 0.2,
            "provider_ready": False,
            "fallback_reason": reason,
            "transaction_count": 0,
            "net_signal": "neutral",
        },
    )


def _is_recent(*, row: dict[str, Any], cutoff: datetime) -> bool:
    event_dt = _parse_datetime(row.get("filingDate") or row.get("transactionDate"))
    if event_dt is None:
        return False
    return _to_utc(event_dt) >= cutoff


def _aggregate_insider_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    buy_count = 0
    sell_count = 0
    buy_value = 0.0
    sell_value = 0.0
    buyers: list[str] = []
    sellers: list[str] = []
    newest_dt: datetime | None = None
    for row in rows:
        tx_type = str(row.get("acquistionOrDisposition") or row.get("acquisitionOrDisposition") or row.get("transactionType") or "").strip().upper()
        shares = _to_float(row.get("securitiesTransacted") or row.get("shares")) or 0.0
        price = _to_float(row.get("price")) or 0.0
        value = shares * price
        holder = str(row.get("reportingName") or row.get("reportingOwnerName") or "").strip()
        event_dt = _parse_datetime(row.get("filingDate") or row.get("transactionDate"))
        if event_dt and (newest_dt is None or event_dt > newest_dt):
            newest_dt = event_dt
        is_buy = tx_type in {"A", "P", "BUY", "ACQUIRE"}
        is_sell = tx_type in {"D", "S", "SELL", "DISPOSE"}
        if is_buy:
            buy_count += 1
            buy_value += value
            if holder:
                buyers.append(holder)
        elif is_sell:
            sell_count += 1
            sell_value += value
            if holder:
                sellers.append(holder)

    denominator = max(1, sell_count)
    ratio = float(buy_count) / float(denominator) if (buy_count + sell_count) > 0 else None
    tx_count = buy_count + sell_count
    net_value = buy_value - sell_value
    net_signal = "bullish" if net_value > 0 else "bearish" if net_value < 0 else "neutral"
    freshness_sec = 7 * 24 * 3600
    delay_sec = None
    if newest_dt is not None:
        delay_sec = int((_to_utc(datetime.now(timezone.utc)) - _to_utc(newest_dt)).total_seconds())
    confidence = 0.75 if tx_count >= 4 else 0.55 if tx_count >= 2 else 0.4
    return {
        "buy_sell_ratio": ratio,
        "transaction_count": tx_count,
        "net_insider_value": net_value,
        "notable_buyers": tuple(sorted(set(buyers))[:5]),
        "notable_sellers": tuple(sorted(set(sellers))[:5]),
        "freshness_sec": freshness_sec,
        "delay_sec": delay_sec,
        "confidence": confidence,
        "net_signal": net_signal,
    }


def _parse_datetime(value: object) -> datetime | None:
    if isinstance(value, str):
        iso = value.replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(iso)
        except ValueError:
            return None
    return None


def _to_float(value: Any) -> float | None:
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


def _fallback_ownership_snapshot(*, symbol: str, as_of: datetime, reason: str) -> InstitutionalOwnershipSnapshot:
    return build_institutional_ownership(
        symbol=symbol,
        as_of=as_of,
        source="ownership_fallback_stub",
        provider_payload={
            "ownership_pct": 0.65,
            "ownership_delta_pct": 0.0,
            "concentration_score": 0.5,
            "sponsorship_score": 0.45,
            "holder_change_balance": 0.0,
            "ownership_signal": "neutral",
            "top_holders": (),
            "delay_days": 45,
            "freshness_sec": 120 * 24 * 3600,
            "confidence": 0.25,
            "provider_ready": False,
            "fallback_reason": reason,
        },
    )


def _aggregate_ownership_rows(*, rows: list[dict[str, Any]], as_of: datetime) -> dict[str, Any]:
    if not rows:
        return {
            "ownership_pct": None,
            "ownership_delta_pct": None,
            "concentration_score": None,
            "top_holders": (),
            "holder_change_balance": 0.0,
            "sponsorship_score": 0.45,
            "ownership_signal": "neutral",
            "delay_days": 45,
            "delay_sec": 45 * 24 * 3600,
            "confidence": 0.25,
        }
    holders: list[tuple[str, float]] = []
    ownership_values: list[float] = []
    increased = 0
    decreased = 0
    new_positions = 0
    sold_out = 0
    latest_dt: datetime | None = None
    for row in rows:
        holder = str(row.get("holder") or row.get("investorName") or row.get("name") or "").strip()
        ownership = _to_float(row.get("ownershipPercentage") or row.get("ownership_pct") or row.get("weight")) or 0.0
        if holder:
            holders.append((holder, ownership))
        ownership_values.append(ownership)
        change = str(row.get("changeType") or row.get("change") or row.get("positionChange") or "").strip().lower()
        if "increase" in change:
            increased += 1
        if "decrease" in change:
            decreased += 1
        if "new" in change:
            new_positions += 1
        if "sold" in change or "out" in change:
            sold_out += 1
        filed_at = _parse_datetime(row.get("filingDate") or row.get("date"))
        if filed_at and (latest_dt is None or filed_at > latest_dt):
            latest_dt = filed_at
    total_ownership = sum(ownership_values)
    top_sorted = sorted(holders, key=lambda item: item[1], reverse=True)
    top = tuple(name for name, _ in top_sorted[:5])
    top_share = sum(weight for _, weight in top_sorted[:5])
    concentration = min(1.0, top_share / max(1.0, total_ownership)) if total_ownership > 0 else 0.5
    balance = float((increased + new_positions) - (decreased + sold_out))
    delta_pct = balance / max(1.0, float(len(rows)))
    sponsorship = min(1.0, max(0.0, (total_ownership / max(1.0, float(len(rows)))) / 100.0 + (concentration * 0.35)))
    signal = "bullish" if delta_pct > 0.1 else "bearish" if delta_pct < -0.1 else "neutral"
    delay_days = 45
    delay_sec = delay_days * 24 * 3600
    if latest_dt is not None:
        delay_sec = max(0, int((_to_utc(as_of) - _to_utc(latest_dt)).total_seconds()))
        delay_days = int(delay_sec / 86400)
    confidence = 0.8 if len(rows) >= 8 else 0.65 if len(rows) >= 4 else 0.5
    return {
        "ownership_pct": total_ownership if total_ownership > 0 else None,
        "ownership_delta_pct": delta_pct,
        "concentration_score": concentration,
        "top_holders": top,
        "holder_change_balance": balance,
        "sponsorship_score": sponsorship,
        "ownership_signal": signal,
        "delay_days": delay_days,
        "delay_sec": delay_sec,
        "confidence": confidence,
    }
