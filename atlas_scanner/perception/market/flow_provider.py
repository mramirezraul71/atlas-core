from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import logging
import os
from typing import Protocol

from .flow_normalizer import RawFlowEvent, synthetic_flow_events
from .unusual_whales_provider import UnusualWhalesFlowProvider, config_from_env

logger = logging.getLogger("atlas_scanner.flow.provider")

class FlowEventsProvider(Protocol):
    def fetch_events(self, *, symbol: str, since: datetime, until: datetime) -> tuple[RawFlowEvent, ...]:
        ...


@dataclass(frozen=True)
class SyntheticFlowEventsProvider:
    def fetch_events(self, *, symbol: str, since: datetime, until: datetime) -> tuple[RawFlowEvent, ...]:
        _ = since, until
        return synthetic_flow_events(symbol)


@dataclass(frozen=True)
class NoOpFlowEventsProvider:
    def fetch_events(self, *, symbol: str, since: datetime, until: datetime) -> tuple[RawFlowEvent, ...]:
        _ = symbol, since, until
        return ()


@dataclass(frozen=True)
class FlowProviderResolution:
    provider_name: str
    provider: FlowEventsProvider
    fallback_used: bool = False
    reason: str | None = None


def resolve_flow_provider(
    *,
    use_real_provider: bool | None = None,
    provider_name: str | None = None,
    fallback_name: str | None = None,
) -> FlowProviderResolution:
    selected = (provider_name or os.getenv("ATLAS_FLOW_PROVIDER", "")).strip().lower()
    if not selected:
        selected = "unusual_whales" if use_real_provider else "synthetic"
    fallback = (fallback_name or os.getenv("ATLAS_FLOW_PROVIDER_FALLBACK", "synthetic")).strip().lower()
    if selected == "synthetic":
        return FlowProviderResolution(provider_name="synthetic", provider=SyntheticFlowEventsProvider())
    if selected == "noop":
        return FlowProviderResolution(provider_name="noop", provider=NoOpFlowEventsProvider())
    if selected == "unusual_whales":
        uw_config = config_from_env()
        if uw_config is not None:
            return FlowProviderResolution(
                provider_name="unusual_whales",
                provider=UnusualWhalesFlowProvider(config=uw_config),
            )
        logger.warning("flow provider unusual_whales unavailable: missing API key")
        fallback_provider = _resolve_fallback(fallback)
        return FlowProviderResolution(
            provider_name=fallback_provider[0],
            provider=fallback_provider[1],
            fallback_used=True,
            reason="unusual_whales_missing_api_key",
        )
    logger.warning("unknown flow provider '%s', falling back to synthetic", selected)
    return FlowProviderResolution(
        provider_name="synthetic",
        provider=SyntheticFlowEventsProvider(),
        fallback_used=True,
        reason=f"unknown_provider:{selected}",
    )


def _resolve_fallback(name: str) -> tuple[str, FlowEventsProvider]:
    if name == "noop":
        return "noop", NoOpFlowEventsProvider()
    return "synthetic", SyntheticFlowEventsProvider()
