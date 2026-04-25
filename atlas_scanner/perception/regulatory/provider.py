from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import os
from typing import Any
from typing import Protocol

import requests

from atlas_scanner.contracts import RegulatoryEventSnapshot
from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from atlas_scanner.perception.regulatory.pipeline import build_regulatory_event_context


class RegulatoryEventProvider(Protocol):
    def fetch(self, *, symbol_or_scope: str, as_of: datetime) -> RegulatoryEventSnapshot:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


@dataclass
class StubRegulatoryEventProvider:
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "regulatory_stub"}

    def fetch(self, *, symbol_or_scope: str, as_of: datetime) -> RegulatoryEventSnapshot:
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "regulatory_stub",
            "provider_ready": False,
            "fallback_active": True,
        }
        return build_regulatory_event_context(
            symbol_or_scope=symbol_or_scope,
            as_of=as_of,
            source="regulatory_stub",
            provider_payload={
                "event_type": "none",
                "severity": "low",
                "overhang_score": 0.0,
                "filing_count": 0,
                "confidence": 0.2,
                "provider_ready": False,
            },
        )


@dataclass(frozen=True)
class FmpRegulatoryProviderConfig:
    api_key: str
    base_url: str = "https://financialmodelingprep.com/stable/sec-filings-search/symbol"
    timeout_sec: float = 6.0
    lookback_days: int = 30
    max_rows: int = 60


@dataclass
class FmpRegulatoryEventProvider:
    config: FmpRegulatoryProviderConfig
    last_diagnostics: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "regulatory_fmp"}

    def fetch(self, *, symbol_or_scope: str, as_of: datetime) -> RegulatoryEventSnapshot:
        breaker = resolve_provider_circuit_breaker("regulatory:fmp")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "regulatory_fmp",
                "error": "circuit_open",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol_or_scope=symbol_or_scope, as_of=as_of, reason="circuit_open")

        params = {
            "symbol": symbol_or_scope.upper(),
            "apikey": self.config.api_key,
            "limit": max(1, self.config.max_rows),
            "page": 0,
        }
        try:
            response = requests.get(self.config.base_url, params=params, timeout=self.config.timeout_sec)
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "regulatory_fmp",
                "error": str(exc),
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol_or_scope=symbol_or_scope, as_of=as_of, reason="request_exception")

        if response.status_code >= 400:
            breaker.record_failure(f"http_{response.status_code}")
            self.last_diagnostics = {
                "status": "error",
                "provider": "regulatory_fmp",
                "error": f"http_{response.status_code}",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol_or_scope=symbol_or_scope, as_of=as_of, reason=f"http_{response.status_code}")

        payload = response.json()
        if not isinstance(payload, list):
            breaker.record_failure("invalid_payload")
            self.last_diagnostics = {
                "status": "error",
                "provider": "regulatory_fmp",
                "error": "invalid_payload",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return _fallback_snapshot(symbol_or_scope=symbol_or_scope, as_of=as_of, reason="invalid_payload")

        cutoff = _to_utc(as_of) - timedelta(days=max(1, self.config.lookback_days))
        selected = [
            row for row in payload[: max(1, self.config.max_rows)]
            if isinstance(row, dict) and _is_recent(row=row, cutoff=cutoff)
        ]
        agg = _aggregate_regulatory_rows(selected, as_of=as_of)
        provider_ready = agg["event_type"] is not None and agg["confidence"] >= 0.4
        if provider_ready:
            breaker.record_success()
        else:
            breaker.record_failure("missing_regulatory_signal")
        self.last_diagnostics = {
            "status": "ok" if provider_ready else "degraded",
            "provider": "regulatory_fmp",
            "provider_ready": provider_ready,
            "fallback_active": not provider_ready,
            "rows": len(payload),
            "selected_rows": len(selected),
            "lookback_days": self.config.lookback_days,
            "circuit_breaker": _cb_dict(breaker.snapshot()),
        }
        if not provider_ready:
            return _fallback_snapshot(symbol_or_scope=symbol_or_scope, as_of=as_of, reason="missing_regulatory_signal")

        return build_regulatory_event_context(
            symbol_or_scope=symbol_or_scope,
            as_of=as_of,
            source="regulatory_fmp",
            provider_payload={
                "event_type": agg["event_type"],
                "severity": agg["severity"],
                "overhang_score": agg["overhang_score"],
                "freshness_sec": agg["freshness_sec"],
                "delay_sec": agg["delay_sec"],
                "confidence": agg["confidence"],
                "provider_ready": True,
                "filing_count": agg["filing_count"],
                "latest_forms": agg["latest_forms"],
            },
        )


def resolve_regulatory_provider() -> RegulatoryEventProvider:
    provider_name = os.getenv("ATLAS_REGULATORY_PROVIDER", "stub").strip().lower()
    if provider_name in {"fmp", "financialmodelingprep"}:
        key = os.getenv("ATLAS_FMP_API_KEY", "").strip()
        if key:
            timeout = float(os.getenv("ATLAS_REGULATORY_TIMEOUT_SEC", "6.0"))
            lookback_days = int(os.getenv("ATLAS_REGULATORY_LOOKBACK_DAYS", "30"))
            max_rows = int(os.getenv("ATLAS_REGULATORY_MAX_ROWS", "60"))
            base_url = os.getenv(
                "ATLAS_REGULATORY_BASE_URL",
                "https://financialmodelingprep.com/stable/sec-filings-search/symbol",
            )
            return FmpRegulatoryEventProvider(
                config=FmpRegulatoryProviderConfig(
                    api_key=key,
                    timeout_sec=timeout,
                    lookback_days=lookback_days,
                    max_rows=max_rows,
                    base_url=base_url,
                )
            )
    return StubRegulatoryEventProvider()


def _fallback_snapshot(*, symbol_or_scope: str, as_of: datetime, reason: str) -> RegulatoryEventSnapshot:
    return build_regulatory_event_context(
        symbol_or_scope=symbol_or_scope,
        as_of=as_of,
        source="regulatory_fallback_stub",
        provider_payload={
            "event_type": "none",
            "severity": "low",
            "overhang_score": 0.0,
            "confidence": 0.25,
            "provider_ready": False,
            "fallback_reason": reason,
            "filing_count": 0,
        },
    )


def _is_recent(*, row: dict[str, Any], cutoff: datetime) -> bool:
    filed_at = _parse_datetime(row.get("fillingDate") or row.get("acceptedDate") or row.get("date"))
    if filed_at is None:
        return False
    return _to_utc(filed_at) >= cutoff


def _aggregate_regulatory_rows(rows: list[dict[str, Any]], *, as_of: datetime) -> dict[str, Any]:
    if not rows:
        return {
            "event_type": None,
            "severity": "low",
            "overhang_score": 0.0,
            "freshness_sec": 7 * 24 * 3600,
            "delay_sec": None,
            "confidence": 0.2,
            "filing_count": 0,
            "latest_forms": (),
        }
    forms = [str(row.get("finalLink") or row.get("formType") or row.get("type") or "unknown") for row in rows]
    risk_forms = [form for form in forms if any(token in form.upper() for token in ("8-K", "10-K", "10-Q", "S-1", "424B"))]
    filing_count = len(rows)
    risk_ratio = len(risk_forms) / filing_count if filing_count > 0 else 0.0
    overhang_score = min(1.0, 0.2 + (risk_ratio * 0.7))
    severity = "critical" if overhang_score >= 0.85 else "high" if overhang_score >= 0.65 else "medium" if overhang_score >= 0.4 else "low"
    latest_dt = _parse_datetime(rows[0].get("fillingDate") or rows[0].get("acceptedDate") or rows[0].get("date"))
    delay_sec = None
    if latest_dt is not None:
        delay_sec = int((_to_utc(as_of) - _to_utc(latest_dt)).total_seconds())
    event_type = "sec_filing_cluster" if filing_count >= 3 else "sec_material_filing"
    confidence = 0.75 if filing_count >= 4 else 0.6 if filing_count >= 2 else 0.45
    return {
        "event_type": event_type,
        "severity": severity,
        "overhang_score": overhang_score,
        "freshness_sec": 14 * 24 * 3600,
        "delay_sec": delay_sec,
        "confidence": confidence,
        "filing_count": filing_count,
        "latest_forms": tuple(forms[:5]),
    }


def _parse_datetime(value: object) -> datetime | None:
    if isinstance(value, str):
        iso = value.replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(iso)
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
