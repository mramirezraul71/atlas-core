from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import OperationalContextSnapshot


def build_operational_context(
    *,
    symbol: str,
    runtime_mode: str = "paper",
    market_session: str = "regular",
    provider_status: Mapping[str, str] | None = None,
    provider_latency_ms: Mapping[str, int] | None = None,
    vision_available: bool = False,
    operator_present: bool = True,
    meta: Mapping[str, Any] | None = None,
    as_of: datetime | None = None,
) -> OperationalContextSnapshot:
    return OperationalContextSnapshot(
        symbol=symbol.upper(),
        as_of=as_of or datetime.now(timezone.utc),
        market_session=market_session,
        runtime_mode=runtime_mode,
        vision_available=vision_available,
        operator_present=operator_present,
        provider_status=dict(provider_status or {}),
        provider_latency_ms=dict(provider_latency_ms or {}),
        meta=dict(meta or {}),
    )
