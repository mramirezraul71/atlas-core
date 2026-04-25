from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import EconomicRecentEvent, EconomicUpcomingEvent, MacroContextSnapshot


def build_macro_context(
    *,
    scope: str,
    provider_payload: Mapping[str, Any] | None = None,
    source: str = "macro_stub",
    as_of: datetime | None = None,
) -> MacroContextSnapshot:
    payload = dict(provider_payload or {})
    upcoming_events = _parse_upcoming_events(payload.get("upcoming_events"))
    recent_events = _parse_recent_events(payload.get("recent_events"))
    high_impact_flag = bool(payload.get("high_impact_flag", False))
    if not high_impact_flag:
        high_impact_flag = any(event.impact_level == "high" for event in upcoming_events)
    calendar_risk_score = _calendar_risk_score(
        upcoming_events=upcoming_events,
        recent_events=recent_events,
        as_of=as_of or datetime.now(timezone.utc),
        scope=scope,
    )
    volatility_window = _calendar_volatility_window(
        upcoming_events=upcoming_events,
        recent_events=recent_events,
        as_of=as_of or datetime.now(timezone.utc),
    )
    recent_surprise_significant = any(abs(event.surprise or 0.0) >= 0.5 for event in recent_events)
    macro_calendar_clear = (not volatility_window) and calendar_risk_score <= 0.25
    confidence = float(payload.get("confidence", 0.35))
    provider_ready = bool(payload.get("provider_ready", bool(payload)))
    quality_flags = {
        "provider_ready": provider_ready,
        "is_stub": not provider_ready,
        "high_impact_flag": high_impact_flag,
        "calendar_volatility_window": volatility_window,
        "calendar_risk_score": calendar_risk_score,
        "recent_surprise_significant": recent_surprise_significant,
        "macro_calendar_clear": macro_calendar_clear,
    }
    return MacroContextSnapshot(
        scope=scope,
        as_of=as_of or datetime.now(timezone.utc),
        source=source,
        upcoming_event=payload.get("upcoming_event"),
        recent_event=payload.get("recent_event"),
        upcoming_events=upcoming_events,
        recent_events=recent_events,
        high_impact_flag=high_impact_flag,
        calendar_risk_score=calendar_risk_score,
        calendar_volatility_window=volatility_window,
        sector_sensitivity=_optional_float(payload.get("sector_sensitivity")),
        freshness_sec=_optional_int(payload.get("freshness_sec")),
        delay_sec=_optional_int(payload.get("delay_sec")),
        confidence=max(0.0, min(1.0, confidence)),
        quality_flags=quality_flags,
        meta={k: v for k, v in payload.items() if k not in {"upcoming_event", "recent_event"}},
    )


def _optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _optional_int(value: object) -> int | None:
    if isinstance(value, int):
        return value
    return None


def _parse_upcoming_events(raw: object) -> tuple[EconomicUpcomingEvent, ...]:
    if not isinstance(raw, (list, tuple)):
        return ()
    events: list[EconomicUpcomingEvent] = []
    for item in raw:
        if isinstance(item, EconomicUpcomingEvent):
            events.append(item)
            continue
        if not isinstance(item, Mapping):
            continue
        release_dt = item.get("release_datetime")
        if not isinstance(release_dt, datetime):
            continue
        impact = str(item.get("impact_level", "medium")).strip().lower()
        if impact not in {"low", "medium", "high"}:
            impact = "medium"
        sectors_raw = item.get("affected_sectors", ())
        sectors = tuple(str(sector) for sector in sectors_raw) if isinstance(sectors_raw, (list, tuple)) else ()
        events.append(
            EconomicUpcomingEvent(
                event_name=str(item.get("event_name", "macro_event")),
                release_datetime=release_dt,
                impact_level=impact,  # type: ignore[arg-type]
                affected_sectors=sectors,
                consensus=_optional_float(item.get("consensus")),
                previous_value=_optional_float(item.get("previous_value")),
            )
        )
    return tuple(sorted(events, key=lambda event: event.release_datetime))


def _parse_recent_events(raw: object) -> tuple[EconomicRecentEvent, ...]:
    if not isinstance(raw, (list, tuple)):
        return ()
    events: list[EconomicRecentEvent] = []
    for item in raw:
        if isinstance(item, EconomicRecentEvent):
            events.append(item)
            continue
        if not isinstance(item, Mapping):
            continue
        release_dt = item.get("release_datetime")
        if not isinstance(release_dt, datetime):
            continue
        impact = str(item.get("impact_level", "medium")).strip().lower()
        if impact not in {"low", "medium", "high"}:
            impact = "medium"
        sectors_raw = item.get("affected_sectors", ())
        sectors = tuple(str(sector) for sector in sectors_raw) if isinstance(sectors_raw, (list, tuple)) else ()
        events.append(
            EconomicRecentEvent(
                event_name=str(item.get("event_name", "macro_event")),
                release_datetime=release_dt,
                actual_value=_optional_float(item.get("actual_value")),
                consensus=_optional_float(item.get("consensus")),
                surprise=_optional_float(item.get("surprise")),
                impact_level=impact,  # type: ignore[arg-type]
                affected_sectors=sectors,
            )
        )
    return tuple(sorted(events, key=lambda event: event.release_datetime, reverse=True))


def _calendar_risk_score(
    *,
    upcoming_events: tuple[EconomicUpcomingEvent, ...],
    recent_events: tuple[EconomicRecentEvent, ...],
    as_of: datetime,
    scope: str,
) -> float:
    risk = 0.0
    now = as_of
    scope_upper = scope.upper()
    for event in upcoming_events:
        delta_h = max(0.0, (event.release_datetime - now).total_seconds() / 3600.0)
        impact = {"low": 0.05, "medium": 0.15, "high": 0.35}.get(event.impact_level, 0.15)
        proximity = 1.0 if delta_h <= 2 else 0.75 if delta_h <= 12 else 0.4
        sector_boost = 1.2 if _scope_matches_sector(scope_upper, event.affected_sectors) else 1.0
        risk += impact * proximity * sector_boost
    if len([event for event in upcoming_events if event.impact_level in {"medium", "high"}]) >= 3:
        risk += 0.15
    for event in recent_events:
        surprise = abs(event.surprise or 0.0)
        if surprise >= 0.5:
            risk += 0.2 if event.impact_level == "high" else 0.1
    return max(0.0, min(1.0, risk))


def _calendar_volatility_window(
    *,
    upcoming_events: tuple[EconomicUpcomingEvent, ...],
    recent_events: tuple[EconomicRecentEvent, ...],
    as_of: datetime,
) -> bool:
    high_soon = any(
        event.impact_level == "high" and (event.release_datetime - as_of).total_seconds() <= 2 * 3600
        for event in upcoming_events
    )
    medium_cluster = len(
        [
            event
            for event in upcoming_events
            if event.impact_level in {"medium", "high"} and (event.release_datetime - as_of).total_seconds() <= 12 * 3600
        ]
    ) >= 2
    recent_surprise = any(
        abs(event.surprise or 0.0) >= 0.5 and (as_of - event.release_datetime).total_seconds() <= 2 * 3600
        for event in recent_events
    )
    return high_soon or medium_cluster or recent_surprise


def _scope_matches_sector(scope_upper: str, sectors: tuple[str, ...]) -> bool:
    if scope_upper in {"SPY", "QQQ", "IWM", "DIA"}:
        return True
    joined = " ".join(sectors).lower()
    if scope_upper in {"XLF"} and "financial" in joined:
        return True
    if scope_upper in {"XLK", "QQQ"} and ("technology" in joined or "rates" in joined):
        return True
    if scope_upper in {"XLY", "XLP"} and "consumer" in joined:
        return True
    return False
