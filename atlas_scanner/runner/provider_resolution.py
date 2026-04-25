from __future__ import annotations

import logging
from dataclasses import dataclass, field
from time import time
from typing import Any, Mapping

from atlas_scanner.data.dummy_gamma_oi import DummyGammaOIProvider
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider
from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider

logger = logging.getLogger("atlas_scanner.providers")


@dataclass(frozen=True)
class OfflineProviders:
    vol_macro_provider: VolMacroProvider
    gamma_oi_provider: GammaOIProvider


def resolve_offline_providers(
    vol_macro_provider: VolMacroProvider | None = None,
    gamma_oi_provider: GammaOIProvider | None = None,
) -> OfflineProviders:
    return OfflineProviders(
        vol_macro_provider=vol_macro_provider or OpenBBVolMacroProvider(),
        gamma_oi_provider=gamma_oi_provider or DummyGammaOIProvider(),
    )


def is_empty_vol_data(data: VolData) -> bool:
    return not data.iv_history and data.iv_current is None and not data.rv_annualized


def is_empty_macro_data(data: MacroData) -> bool:
    return data.vix is None and data.macro_regime is None and data.seasonal_factor is None


def is_empty_gamma_data(data: GammaData) -> bool:
    return not data.strikes and data.net_gex is None


def is_empty_oi_flow_data(data: OIFlowData) -> bool:
    return (
        data.oi_change_1d_pct is None
        and data.call_put_volume_ratio is None
        and data.volume_imbalance is None
        and data.call_volume is None
        and data.put_volume is None
        and not data.meta
    )


@dataclass(frozen=True)
class ProviderHealthStats:
    provider_name: str
    source_type: str
    total_calls: int = 0
    success_calls: int = 0
    error_calls: int = 0
    avg_latency_ms: float = 0.0
    last_status: str = "unknown"
    last_error: str | None = None
    last_updated_at: float | None = None
    fallback_active: bool = False
    provider_ready: bool = True
    consecutive_errors: int = 0
    error_rate: float = 0.0
    circuit_state: str | None = None
    availability: str | None = None
    details: Mapping[str, Any] = field(default_factory=dict)
    latency_window: tuple[int, ...] = ()


@dataclass
class RealtimeProviderRegistry:
    _stats: dict[tuple[str, str], ProviderHealthStats] = field(default_factory=dict)

    def record(
        self,
        *,
        source_type: str,
        provider_name: str,
        status: str,
        latency_ms: int,
        error: str | None = None,
        extras: Mapping[str, Any] | None = None,
    ) -> None:
        key = (source_type, provider_name)
        current = self._stats.get(
            key,
            ProviderHealthStats(provider_name=provider_name, source_type=source_type),
        )
        total_calls = current.total_calls + 1
        success_calls = current.success_calls + (1 if status == "ok" else 0)
        error_calls = current.error_calls + (1 if status == "error" else 0)
        avg_latency = ((current.avg_latency_ms * current.total_calls) + latency_ms) / total_calls
        updated = ProviderHealthStats(
            provider_name=provider_name,
            source_type=source_type,
            total_calls=total_calls,
            success_calls=success_calls,
            error_calls=error_calls,
            avg_latency_ms=avg_latency,
            last_status=status,
            last_error=error,
            last_updated_at=time(),
            fallback_active=bool((extras or {}).get("fallback_active", current.fallback_active)),
            provider_ready=bool((extras or {}).get("provider_ready", current.provider_ready)),
            consecutive_errors=(0 if status == "ok" else current.consecutive_errors + 1),
            error_rate=round(error_calls / total_calls, 4),
            circuit_state=str((extras or {}).get("circuit_state", current.circuit_state or "")) or None,
            availability=str((extras or {}).get("availability", current.availability or "")) or None,
            details=dict((extras or {}).get("details", current.details)),
            latency_window=tuple((list(current.latency_window[-19:]) + [int(latency_ms)])),
        )
        self._stats[key] = updated
        if status != "ok":
            logger.warning(
                "provider degraded source=%s provider=%s status=%s latency_ms=%s error=%s",
                source_type,
                provider_name,
                status,
                latency_ms,
                error or "",
            )

    def resolve_provider(
        self,
        *,
        source_type: str,
        preferred: tuple[str, ...],
    ) -> str:
        for provider in preferred:
            stat = self._stats.get((source_type, provider))
            if stat is None:
                return provider
            if stat.last_status != "error":
                return provider
        return preferred[0] if preferred else "none"

    def snapshot(self) -> dict[str, dict[str, float | int | str | None]]:
        return {
            f"{source}:{provider}": normalize_provider_diagnostics(source=source, provider=provider, stats=stats)
            for (source, provider), stats in self._stats.items()
        }


def normalize_provider_diagnostics(
    *,
    source: str,
    provider: str,
    stats: ProviderHealthStats,
) -> dict[str, float | int | str | bool | None | dict[str, Any]]:
    latency_samples = sorted(stats.latency_window)
    p95 = 0.0
    if latency_samples:
        idx = min(len(latency_samples) - 1, int(round((len(latency_samples) - 1) * 0.95)))
        p95 = float(latency_samples[idx])
    burst_error = stats.consecutive_errors >= 3
    circuit_open = str(stats.circuit_state or "").lower() == "open"
    return {
        "source_type": source,
        "provider_name": provider,
        "provider_ready": stats.provider_ready,
        "fallback_active": stats.fallback_active,
        "circuit_state": stats.circuit_state,
        "latency_ms": round(stats.avg_latency_ms, 2),
        "p95_latency_ms": round(p95, 2),
        "consecutive_errors": stats.consecutive_errors,
        "last_error": stats.last_error,
        "availability": stats.availability,
        "error_rate": stats.error_rate,
        "total_calls": stats.total_calls,
        "success_calls": stats.success_calls,
        "error_calls": stats.error_calls,
        "last_status": stats.last_status,
        "last_updated_at": stats.last_updated_at,
        "burst_error_indicator": burst_error,
        "circuit_open_indicator": circuit_open,
        "active_fallback_indicator": bool(stats.fallback_active),
        "details": dict(stats.details),
    }

