from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import RegulatoryEventSnapshot


def build_regulatory_event_context(
    *,
    symbol_or_scope: str,
    provider_payload: Mapping[str, Any] | None = None,
    source: str = "regulatory_stub",
    as_of: datetime | None = None,
) -> RegulatoryEventSnapshot:
    payload = dict(provider_payload or {})
    severity_raw = str(payload.get("severity", "low")).strip().lower()
    if severity_raw not in {"low", "medium", "high", "critical"}:
        severity_raw = "low"
    confidence = float(payload.get("confidence", 0.25))
    provider_ready = bool(payload.get("provider_ready", bool(payload)))
    return RegulatoryEventSnapshot(
        symbol_or_scope=symbol_or_scope,
        as_of=as_of or datetime.now(timezone.utc),
        source=source,
        event_type=str(payload.get("event_type")) if payload.get("event_type") is not None else None,
        severity=severity_raw,  # type: ignore[arg-type]
        overhang_score=_optional_float(payload.get("overhang_score")),
        freshness_sec=_optional_int(payload.get("freshness_sec")),
        delay_sec=_optional_int(payload.get("delay_sec")),
        confidence=max(0.0, min(1.0, confidence)),
        quality_flags={"provider_ready": provider_ready, "is_stub": not provider_ready},
        meta={k: v for k, v in payload.items() if k not in {"event_type", "severity", "overhang_score"}},
    )


def _optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _optional_int(value: object) -> int | None:
    if isinstance(value, int):
        return value
    return None
