from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import InstitutionalOwnershipSnapshot, InsiderTradingSnapshot


def build_institutional_ownership(
    *,
    symbol: str,
    provider_payload: Mapping[str, Any] | None = None,
    source: str = "institutional_stub",
    as_of: datetime | None = None,
) -> InstitutionalOwnershipSnapshot:
    payload = dict(provider_payload or {})
    confidence = float(payload.get("confidence", 0.3))
    provider_ready = bool(payload.get("provider_ready", bool(payload)))
    quality_flags = {
        "provider_ready": provider_ready,
        "is_stub": not provider_ready,
    }
    return InstitutionalOwnershipSnapshot(
        symbol=symbol.upper(),
        as_of=as_of or datetime.now(timezone.utc),
        source=source,
        ownership_pct=_optional_float(payload.get("ownership_pct")),
        ownership_delta_pct=_optional_float(payload.get("ownership_delta_pct")),
        concentration_score=_optional_float(payload.get("concentration_score")),
        freshness_sec=_optional_int(payload.get("freshness_sec")),
        delay_sec=_optional_int(payload.get("delay_sec")),
        confidence=max(0.0, min(1.0, confidence)),
        quality_flags=quality_flags,
        meta={k: v for k, v in payload.items() if k not in {"ownership_pct", "ownership_delta_pct", "concentration_score"}},
    )


def build_insider_trading(
    *,
    symbol: str,
    provider_payload: Mapping[str, Any] | None = None,
    source: str = "insider_stub",
    as_of: datetime | None = None,
) -> InsiderTradingSnapshot:
    payload = dict(provider_payload or {})
    buyers = tuple(payload.get("notable_buyers", ())) if isinstance(payload.get("notable_buyers"), (list, tuple)) else ()
    sellers = tuple(payload.get("notable_sellers", ())) if isinstance(payload.get("notable_sellers"), (list, tuple)) else ()
    confidence = float(payload.get("confidence", 0.3))
    provider_ready = bool(payload.get("provider_ready", bool(payload)))
    quality_flags = {
        "provider_ready": provider_ready,
        "is_stub": not provider_ready,
    }
    return InsiderTradingSnapshot(
        symbol=symbol.upper(),
        as_of=as_of or datetime.now(timezone.utc),
        source=source,
        net_insider_value=_optional_float(payload.get("net_insider_value")),
        buy_sell_ratio=_optional_float(payload.get("buy_sell_ratio")),
        notable_buyers=buyers,
        notable_sellers=sellers,
        freshness_sec=_optional_int(payload.get("freshness_sec")),
        delay_sec=_optional_int(payload.get("delay_sec")),
        confidence=max(0.0, min(1.0, confidence)),
        quality_flags=quality_flags,
        meta={k: v for k, v in payload.items() if k not in {"net_insider_value", "buy_sell_ratio"}},
    )


def _optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _optional_int(value: object) -> int | None:
    if isinstance(value, int):
        return value
    return None
