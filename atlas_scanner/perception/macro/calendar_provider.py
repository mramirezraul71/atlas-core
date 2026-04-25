from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import os
from typing import Any, Protocol

import requests

from atlas_scanner.contracts import EconomicRecentEvent, EconomicUpcomingEvent
from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker

_TE_BASE_URL = "https://api.tradingeconomics.com/calendar"


class EconomicCalendarProvider(Protocol):
    def fetch_calendar(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicUpcomingEvent, ...]:
        ...

    def fetch_recent_events(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicRecentEvent, ...]:
        ...


@dataclass(frozen=True)
class StubCalendarProvider:
    def fetch_calendar(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicUpcomingEvent, ...]:
        _ = min_impact
        now = _utc_now()
        return (
            EconomicUpcomingEvent(
                event_name="FOMC Rate Decision",
                release_datetime=min(to_date, now + timedelta(hours=6)),
                impact_level="high",
                affected_sectors=("financials", "broad_market"),
                consensus=None,
                previous_value=None,
            ),
            EconomicUpcomingEvent(
                event_name="Nonfarm Payrolls",
                release_datetime=min(to_date, now + timedelta(hours=28)),
                impact_level="high",
                affected_sectors=("broad_market", "industrials"),
                consensus=None,
                previous_value=None,
            ),
        )

    def fetch_recent_events(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicRecentEvent, ...]:
        _ = from_date, to_date, min_impact
        now = _utc_now()
        return (
            EconomicRecentEvent(
                event_name="CPI",
                release_datetime=now - timedelta(hours=1),
                actual_value=3.4,
                consensus=3.2,
                surprise=0.2,
                impact_level="high",
                affected_sectors=("consumer", "rates", "broad_market"),
            ),
        )


@dataclass(frozen=True)
class ForexFactoryCalendarProvider:
    def fetch_calendar(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicUpcomingEvent, ...]:
        # Keep this provider semi-real and safe (no scraping in core runtime yet).
        _ = from_date, to_date, min_impact
        return ()

    def fetch_recent_events(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicRecentEvent, ...]:
        _ = from_date, to_date, min_impact
        return ()


@dataclass
class TradingEconomicsCalendarProvider:
    api_key: str
    base_url: str = _TE_BASE_URL
    timeout_sec: float = 6.0
    last_diagnostics: dict[str, Any] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.last_diagnostics is None:
            self.last_diagnostics = {"status": "idle", "provider": "tradingeconomics_calendar"}

    def fetch_calendar(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicUpcomingEvent, ...]:
        start = _as_date(from_date)
        end = _as_date(to_date)
        url = f"{self.base_url}/{start}/{end}"
        params = {"c": self.api_key}
        breaker = resolve_provider_circuit_breaker("calendar:tradingeconomics")
        if not breaker.allow_request():
            snap = breaker.snapshot()
            self.last_diagnostics = {
                "status": "error",
                "provider": "tradingeconomics_calendar",
                "error": "circuit_open",
                "circuit_breaker": _cb_dict(snap),
            }
            return ()
        try:
            response = requests.get(url, params=params, timeout=self.timeout_sec)
            if response.status_code >= 400:
                breaker.record_failure(f"http_{response.status_code}")
                self.last_diagnostics = {
                    "status": "error",
                    "provider": "tradingeconomics_calendar",
                    "error": f"http_{response.status_code}",
                    "circuit_breaker": _cb_dict(breaker.snapshot()),
                }
                return ()
            payload = response.json()
            if not isinstance(payload, list):
                breaker.record_failure("invalid_payload")
                return ()
            min_rank = _impact_rank(min_impact)
            rows: list[EconomicUpcomingEvent] = []
            for item in payload:
                if not isinstance(item, dict):
                    continue
                impact = _normalize_impact(item.get("Importance") or item.get("importance") or item.get("Impact"))
                if _impact_rank(impact) < min_rank:
                    continue
                release_dt = _parse_datetime(item.get("Date") or item.get("date"))
                if release_dt is None:
                    continue
                rows.append(
                    EconomicUpcomingEvent(
                        event_name=str(item.get("Event") or item.get("event") or "macro_event"),
                        release_datetime=release_dt,
                        impact_level=impact,
                        affected_sectors=_affected_sectors_from_event(str(item.get("Event") or "")),
                        consensus=_to_float(item.get("Forecast") or item.get("forecast")),
                        previous_value=_to_float(item.get("Previous") or item.get("previous")),
                    )
                )
            breaker.record_success()
            self.last_diagnostics = {
                "status": "ok" if rows else "empty",
                "provider": "tradingeconomics_calendar",
                "events": len(rows),
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return tuple(sorted(rows, key=lambda event: event.release_datetime))
        except Exception as exc:
            breaker.record_failure(str(exc))
            self.last_diagnostics = {
                "status": "error",
                "provider": "tradingeconomics_calendar",
                "error": str(exc),
                "circuit_breaker": _cb_dict(breaker.snapshot()),
            }
            return ()

    def fetch_recent_events(
        self,
        *,
        from_date: datetime,
        to_date: datetime,
        min_impact: str = "medium",
    ) -> tuple[EconomicRecentEvent, ...]:
        start = _as_date(from_date)
        end = _as_date(to_date)
        url = f"{self.base_url}/{start}/{end}"
        params = {"c": self.api_key}
        breaker = resolve_provider_circuit_breaker("calendar:tradingeconomics")
        if not breaker.allow_request():
            return ()
        try:
            response = requests.get(url, params=params, timeout=self.timeout_sec)
            if response.status_code >= 400:
                breaker.record_failure(f"http_{response.status_code}")
                return ()
            payload = response.json()
            if not isinstance(payload, list):
                breaker.record_failure("invalid_payload")
                return ()
            min_rank = _impact_rank(min_impact)
            recent: list[EconomicRecentEvent] = []
            now = _utc_now()
            for item in payload:
                if not isinstance(item, dict):
                    continue
                impact = _normalize_impact(item.get("Importance") or item.get("importance") or item.get("Impact"))
                if _impact_rank(impact) < min_rank:
                    continue
                release_dt = _parse_datetime(item.get("Date") or item.get("date"))
                if release_dt is None or release_dt > now:
                    continue
                actual = _to_float(item.get("Actual") or item.get("actual"))
                consensus = _to_float(item.get("Forecast") or item.get("forecast"))
                previous = _to_float(item.get("Previous") or item.get("previous"))
                surprise = _surprise(actual=actual, consensus=consensus)
                recent.append(
                    EconomicRecentEvent(
                        event_name=str(item.get("Event") or item.get("event") or "macro_event"),
                        release_datetime=release_dt,
                        actual_value=actual if actual is not None else previous,
                        consensus=consensus,
                        surprise=surprise,
                        impact_level=impact,
                        affected_sectors=_affected_sectors_from_event(str(item.get("Event") or "")),
                    )
                )
            breaker.record_success()
            return tuple(sorted(recent, key=lambda event: event.release_datetime, reverse=True))
        except Exception as exc:
            breaker.record_failure(str(exc))
            return ()


def build_recent_events(
    *,
    calendar_events: tuple[EconomicUpcomingEvent, ...],
    as_of: datetime,
    lookback_hours: int,
) -> tuple[EconomicRecentEvent, ...]:
    lower = _to_utc(as_of) - timedelta(hours=max(1, lookback_hours))
    upper = _to_utc(as_of)
    out: list[EconomicRecentEvent] = []
    for event in calendar_events:
        release = _to_utc(event.release_datetime)
        if not (lower <= release <= upper):
            continue
        out.append(
            EconomicRecentEvent(
                event_name=event.event_name,
                release_datetime=event.release_datetime,
                actual_value=event.previous_value,
                consensus=event.consensus,
                surprise=_surprise(actual=event.previous_value, consensus=event.consensus),
                impact_level=event.impact_level,
                affected_sectors=event.affected_sectors,
            )
        )
    return tuple(out)


def resolve_calendar_provider() -> EconomicCalendarProvider:
    selected = os.getenv("ATLAS_ECONOMIC_CALENDAR_PROVIDER", "stub").strip().lower()
    if selected == "tradingeconomics":
        key = os.getenv("ATLAS_TRADING_ECONOMICS_API_KEY", "").strip()
        if key:
            return TradingEconomicsCalendarProvider(api_key=key)
        return StubCalendarProvider()
    if selected == "forexfactory":
        return ForexFactoryCalendarProvider()
    return StubCalendarProvider()


def calendar_window_hours() -> tuple[int, int]:
    lookahead = int(os.getenv("ATLAS_ECONOMIC_CALENDAR_LOOKAHEAD_HOURS", "72"))
    lookback = int(os.getenv("ATLAS_ECONOMIC_CALENDAR_LOOKBACK_HOURS", "24"))
    return max(1, lookahead), max(1, lookback)


def _as_date(value: datetime) -> str:
    return _to_utc(value).strftime("%Y-%m-%d")


def _parse_datetime(value: object) -> datetime | None:
    if isinstance(value, str):
        iso = value.replace("Z", "+00:00")
        try:
            return datetime.fromisoformat(iso)
        except ValueError:
            return None
    return None


def _normalize_impact(value: object) -> str:
    raw = str(value or "medium").strip().lower()
    if raw in {"high", "3"}:
        return "high"
    if raw in {"low", "1"}:
        return "low"
    return "medium"


def _impact_rank(value: str) -> int:
    return {"low": 1, "medium": 2, "high": 3}.get(value, 2)


def _to_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.replace("%", "").replace(",", "").strip())
        except ValueError:
            return None
    return None


def _surprise(*, actual: float | None, consensus: float | None) -> float | None:
    if actual is None or consensus is None:
        return None
    return actual - consensus


def _affected_sectors_from_event(event_name: str) -> tuple[str, ...]:
    name = event_name.lower()
    if "cpi" in name or "inflation" in name:
        return ("consumer", "rates", "broad_market")
    if "fomc" in name or "rate" in name:
        return ("financials", "rates", "broad_market")
    if "payroll" in name or "employment" in name:
        return ("industrials", "consumer", "broad_market")
    if "gdp" in name:
        return ("cyclicals", "broad_market")
    return ("broad_market",)


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


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
