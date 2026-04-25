from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta
import os
from typing import Any, Protocol

import requests

from atlas_scanner.contracts import MacroContextSnapshot
from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from atlas_scanner.perception.macro.calendar_provider import (
    build_recent_events,
    calendar_window_hours,
    resolve_calendar_provider,
)
from atlas_scanner.perception.macro.pipeline import build_macro_context


class MacroContextProvider(Protocol):
    def fetch(self, *, scope: str, as_of: datetime) -> MacroContextSnapshot:
        ...

    @property
    def last_diagnostics(self) -> dict[str, Any]:
        ...


@dataclass
class StubMacroProvider:
    last_diagnostics: dict[str, Any] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "macro_stub"}

    def fetch(self, *, scope: str, as_of: datetime) -> MacroContextSnapshot:
        calendar = resolve_calendar_provider()
        lookahead_h, lookback_h = calendar_window_hours()
        upcoming = calendar.fetch_calendar(
            from_date=as_of - timedelta(hours=lookback_h),
            to_date=as_of + timedelta(hours=lookahead_h),
            min_impact="medium",
        )
        if hasattr(calendar, "fetch_recent_events"):
            recent = calendar.fetch_recent_events(
                from_date=as_of - timedelta(hours=lookback_h),
                to_date=as_of + timedelta(hours=lookahead_h),
                min_impact="medium",
            )
        else:
            recent = build_recent_events(calendar_events=upcoming, as_of=as_of, lookback_hours=lookback_h)
        self.last_diagnostics = {
            "status": "degraded",
            "provider": "macro_stub",
            "provider_ready": False,
            "fallback_active": True,
            "calendar_provider": calendar.__class__.__name__,
            "calendar_diag": getattr(calendar, "last_diagnostics", {}),
        }
        return build_macro_context(
            scope=scope,
            as_of=as_of,
            source="macro_stub",
            provider_payload={
                "upcoming_event": "fomc",
                "recent_event": "cpi_release",
                "high_impact_flag": True,
                "sector_sensitivity": 0.3,
                "confidence": 0.3,
                "provider_ready": False,
                "upcoming_events": upcoming,
                "recent_events": recent,
            },
        )


@dataclass
class FredMacroProvider:
    api_key: str
    base_url: str = "https://api.stlouisfed.org/fred/series/observations"
    timeout_sec: float = 5.0
    series_id: str = "VIXCLS"
    last_diagnostics: dict[str, Any] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "fred_macro"}

    def fetch(self, *, scope: str, as_of: datetime) -> MacroContextSnapshot:
        calendar = resolve_calendar_provider()
        lookahead_h, lookback_h = calendar_window_hours()
        upcoming = calendar.fetch_calendar(
            from_date=as_of - timedelta(hours=lookback_h),
            to_date=as_of + timedelta(hours=lookahead_h),
            min_impact="medium",
        )
        if hasattr(calendar, "fetch_recent_events"):
            recent = calendar.fetch_recent_events(
                from_date=as_of - timedelta(hours=lookback_h),
                to_date=as_of + timedelta(hours=lookahead_h),
                min_impact="medium",
            )
        else:
            recent = build_recent_events(calendar_events=upcoming, as_of=as_of, lookback_hours=lookback_h)
        breaker = resolve_provider_circuit_breaker("macro:fred")
        if not breaker.allow_request():
            self.last_diagnostics = {
                "status": "error",
                "provider": "fred_macro",
                "error": "circuit_open",
                "provider_ready": False,
                "fallback_active": True,
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return build_macro_context(
                scope=scope,
                as_of=as_of,
                source="fred",
                provider_payload={
                    "provider_ready": False,
                    "upcoming_events": upcoming,
                    "recent_events": recent,
                },
            )
        params = {
            "series_id": self.series_id,
            "api_key": self.api_key,
            "file_type": "json",
            "sort_order": "desc",
            "limit": 2,
        }
        try:
            response = requests.get(self.base_url, params=params, timeout=self.timeout_sec)
            if response.status_code >= 400:
                breaker.record_failure(f"http_{response.status_code}")
                self.last_diagnostics = {
                    "status": "error",
                    "provider": "fred_macro",
                    "error": f"http_{response.status_code}",
                    "provider_ready": False,
                    "fallback_active": True,
                    "circuit_breaker": _cb_dict(breaker.snapshot()),
                }
                return build_macro_context(
                    scope=scope,
                    as_of=as_of,
                    source="fred",
                    provider_payload={
                        "provider_ready": False,
                        "upcoming_events": upcoming,
                        "recent_events": recent,
                    },
                )
            payload = response.json()
            observations = payload.get("observations")
            if not isinstance(observations, list) or not observations:
                breaker.record_failure("empty_observations")
                return build_macro_context(
                    scope=scope,
                    as_of=as_of,
                    source="fred",
                    provider_payload={
                        "provider_ready": False,
                        "upcoming_events": upcoming,
                        "recent_events": recent,
                    },
                )
            latest = observations[-1]
            prev = observations[-2] if len(observations) > 1 else latest
            latest_v = _to_float(latest.get("value"))
            prev_v = _to_float(prev.get("value"))
            if latest_v is None:
                breaker.record_failure("missing_latest_value")
                return build_macro_context(
                    scope=scope,
                    as_of=as_of,
                    source="fred",
                    provider_payload={
                        "provider_ready": False,
                        "upcoming_events": upcoming,
                        "recent_events": recent,
                    },
                )
            vix_delta = (latest_v - prev_v) if latest_v is not None and prev_v is not None else 0.0
            high_impact = (latest_v or 0.0) >= 22.0
            breaker.record_success()
            self.last_diagnostics = {
                "status": "ok",
                "provider": "fred_macro",
                "provider_ready": True,
                "fallback_active": False,
                "series_id": self.series_id,
                "latest_value": latest_v,
                "calendar_provider": calendar.__class__.__name__,
                "calendar_diag": getattr(calendar, "last_diagnostics", {}),
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return build_macro_context(
                scope=scope,
                as_of=as_of,
                source="fred",
                provider_payload={
                    "upcoming_event": "macro_release_window",
                    "recent_event": f"{self.series_id}:{latest.get('date')}",
                    "high_impact_flag": high_impact,
                    "sector_sensitivity": min(1.0, max(-1.0, vix_delta / 10.0)),
                    "confidence": 0.7,
                    "provider_ready": True,
                    "vix_level": latest_v,
                    "upcoming_events": upcoming,
                    "recent_events": recent,
                },
            )
        except Exception:
            breaker.record_failure("request_exception")
            self.last_diagnostics = {
                "status": "error",
                "provider": "fred_macro",
                "error": "request_exception",
                "provider_ready": False,
                "fallback_active": True,
                "calendar_provider": calendar.__class__.__name__,
                "calendar_diag": getattr(calendar, "last_diagnostics", {}),
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return build_macro_context(
                scope=scope,
                as_of=as_of,
                source="fred",
                provider_payload={
                    "provider_ready": False,
                    "upcoming_events": upcoming,
                    "recent_events": recent,
                },
            )


def resolve_macro_provider() -> MacroContextProvider:
    selected = os.getenv("ATLAS_MACRO_PROVIDER", "stub").strip().lower()
    if selected in {"openbb", "openbb_calendar"}:
        # OpenBB economic calendar is not wired in this repo yet. Fall back safely.
        selected = "fred"
    if selected == "fred":
        key = os.getenv("ATLAS_FRED_API_KEY", "").strip()
        if key:
            return FredMacroProvider(api_key=key)
    return StubMacroProvider()


def _to_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _cb_dict(snapshot: Any) -> dict[str, Any]:
    return {
        "state": getattr(snapshot, "state", "unknown"),
        "consecutive_failures": getattr(snapshot, "consecutive_failures", 0),
        "failure_threshold": getattr(snapshot, "failure_threshold", 0),
        "cooldown_sec": getattr(snapshot, "cooldown_sec", 0),
        "last_error": getattr(snapshot, "last_error", None),
    }
