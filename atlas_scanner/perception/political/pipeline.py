from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import PoliticalTradingSnapshot


def build_political_trading_context(
    *,
    scope: str,
    provider_payload: Mapping[str, Any] | None = None,
    source: str = "political_stub",
    as_of: datetime | None = None,
) -> PoliticalTradingSnapshot:
    payload = dict(provider_payload or {})
    related = tuple(payload.get("related_tickers", ())) if isinstance(payload.get("related_tickers"), (list, tuple)) else ()
    confidence = float(payload.get("confidence", 0.25))
    provider_ready = bool(payload.get("provider_ready", bool(payload)))
    return PoliticalTradingSnapshot(
        scope=scope,
        as_of=as_of or datetime.now(timezone.utc),
        source=source,
        net_political_flow=_optional_float(payload.get("net_political_flow")),
        related_tickers=related,
        disclosure_lag_days=_optional_int(payload.get("disclosure_lag_days")),
        freshness_sec=_optional_int(payload.get("freshness_sec")),
        delay_sec=_optional_int(payload.get("delay_sec")),
        confidence=max(0.0, min(1.0, confidence)),
        quality_flags={"provider_ready": provider_ready, "is_stub": not provider_ready},
        meta={k: v for k, v in payload.items() if k not in {"net_political_flow", "related_tickers"}},
    )


def _optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _optional_int(value: object) -> int | None:
    if isinstance(value, int):
        return value
    return None
