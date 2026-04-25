from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Mapping

DomainName = str
HorizonName = str

_TTL_SECONDS: dict[DomainName, dict[HorizonName, int]] = {
    "market": {"intraday": 90, "swing": 900, "positional": 3600},
    "flow": {"intraday": 120, "swing": 1800, "positional": 7200},
    "dealer": {"intraday": 90, "swing": 7200, "positional": 0},
    "macro": {"intraday": 900, "swing": 21600, "positional": 86400},
    "insider": {"intraday": 0, "swing": 7 * 86400, "positional": 30 * 86400},
    "ownership": {"intraday": 0, "swing": 30 * 86400, "positional": 120 * 86400},
    "political": {"intraday": 0, "swing": 14 * 86400, "positional": 45 * 86400},
    "regulatory": {"intraday": 3600, "swing": 7 * 86400, "positional": 30 * 86400},
}


@dataclass(frozen=True)
class DomainFreshness:
    domain: str
    horizon: str
    status: str  # usable | stale | degraded | non_operable
    ttl_sec: int
    age_sec: int | None
    delay_sec: int | None
    confidence: float
    weight_multiplier: float
    reason: str | None = None


def evaluate_domain_freshness(
    *,
    domain: str,
    horizon: str,
    as_of: datetime,
    snapshot_as_of: datetime | None,
    delay_sec: int | None,
    confidence: float | None,
    quality_flags: Mapping[str, object] | None,
) -> DomainFreshness:
    ttl_sec = _TTL_SECONDS.get(domain, {}).get(horizon, 900)
    if domain == "macro":
        ttl_sec = _macro_dynamic_ttl(ttl_sec=ttl_sec, horizon=horizon, quality_flags=quality_flags)
    if ttl_sec <= 0:
        return DomainFreshness(
            domain=domain,
            horizon=horizon,
            status="non_operable",
            ttl_sec=ttl_sec,
            age_sec=None,
            delay_sec=delay_sec,
            confidence=_bound_confidence(confidence),
            weight_multiplier=0.0,
            reason=f"{domain}_not_allowed_for_{horizon}",
        )
    age_sec = _age_seconds(as_of=as_of, snapshot_as_of=snapshot_as_of)
    conf = _bound_confidence(confidence)
    is_stub = bool((quality_flags or {}).get("is_stub", False))
    provider_ready = bool((quality_flags or {}).get("provider_ready", True))
    if not provider_ready:
        return DomainFreshness(domain, horizon, "degraded", ttl_sec, age_sec, delay_sec, conf, 0.2, "provider_not_ready")
    if age_sec is None:
        return DomainFreshness(domain, horizon, "degraded", ttl_sec, age_sec, delay_sec, conf, 0.25, "missing_snapshot_timestamp")
    if age_sec > ttl_sec:
        return DomainFreshness(domain, horizon, "stale", ttl_sec, age_sec, delay_sec, conf, 0.0, f"age_sec>{ttl_sec}")
    if is_stub:
        return DomainFreshness(domain, horizon, "degraded", ttl_sec, age_sec, delay_sec, conf, 0.35, "stub_provider")
    return DomainFreshness(domain, horizon, "usable", ttl_sec, age_sec, delay_sec, conf, max(0.2, conf))


def _macro_dynamic_ttl(*, ttl_sec: int, horizon: str, quality_flags: Mapping[str, object] | None) -> int:
    flags = quality_flags or {}
    volatility_window = bool(flags.get("calendar_volatility_window", False))
    recent_surprise = bool(flags.get("recent_surprise_significant", False))
    if volatility_window and horizon == "intraday":
        return min(ttl_sec, 180)
    if volatility_window and horizon == "swing":
        return min(ttl_sec, 1800)
    if recent_surprise and horizon in {"intraday", "swing"}:
        return max(ttl_sec, 3600)
    if bool(flags.get("macro_calendar_clear", False)):
        return max(ttl_sec, int(ttl_sec * 1.5))
    return ttl_sec


def effective_horizon_weights(
    *,
    horizon: str,
    base_weights: Mapping[str, float],
    domain_freshness: Mapping[str, DomainFreshness],
) -> dict[str, float]:
    adjusted: dict[str, float] = {}
    for domain, base in base_weights.items():
        multiplier = domain_freshness.get(domain).weight_multiplier if domain in domain_freshness else 0.0
        adjusted[domain] = max(0.0, base) * max(0.0, multiplier)
    total = sum(adjusted.values())
    if total <= 0:
        return {key: 0.0 for key in base_weights.keys()}
    return {key: value / total for key, value in adjusted.items()}


def _age_seconds(*, as_of: datetime, snapshot_as_of: datetime | None) -> int | None:
    if snapshot_as_of is None:
        return None
    now = _to_utc(as_of)
    snap = _to_utc(snapshot_as_of)
    return max(0, int((now - snap).total_seconds()))


def _to_utc(value: datetime) -> datetime:
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def _bound_confidence(value: float | None) -> float:
    if value is None:
        return 0.0
    if value < 0:
        return 0.0
    if value > 1:
        return 1.0
    return float(value)
