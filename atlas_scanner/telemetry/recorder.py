from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import RadarSignal, RadarSignalBatch


@dataclass(frozen=True)
class RadarTelemetryEvent:
    emitted_at: datetime
    pipeline: str
    symbol: str
    timeframe: str
    sources_used: tuple[str, ...]
    aggregate_score: float
    principal_scenario: str
    principal_conviction_reason: str
    principal_degradation_reason: str | None
    meta: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class InMemoryRadarTelemetryRecorder:
    events: list[RadarTelemetryEvent] = field(default_factory=list)

    def record_batch(
        self,
        batch: RadarSignalBatch,
        *,
        pipeline: str,
        sources_used: tuple[str, ...],
    ) -> tuple[RadarTelemetryEvent, ...]:
        emitted_at = datetime.now(timezone.utc)
        recorded: list[RadarTelemetryEvent] = []
        for signal in batch.signals:
            principal_scenario = _primary_scenario_name(batch, signal)
            event = RadarTelemetryEvent(
                emitted_at=emitted_at,
                pipeline=pipeline,
                symbol=signal.symbol,
                timeframe=signal.timeframe,
                sources_used=sources_used,
                aggregate_score=signal.aggregate_conviction_score,
                principal_scenario=principal_scenario,
                principal_conviction_reason=signal.primary_conviction_reason,
                principal_degradation_reason=signal.primary_degradation_reason,
                meta={
                    "operable": signal.quality.is_operable,
                    "active_domains": _meta_list(signal.meta, "active_domains"),
                    "degraded_domains": _meta_list(signal.meta, "degraded_domains"),
                    "horizon_scores": signal.meta.get("horizon_scores") if isinstance(signal.meta, Mapping) else {},
                    "freshness": signal.meta.get("freshness") if isinstance(signal.meta, Mapping) else {},
                    "effective_weights": signal.meta.get("effective_weights") if isinstance(signal.meta, Mapping) else {},
                    "macro_calendar": signal.meta.get("macro_calendar") if isinstance(signal.meta, Mapping) else {},
                    "provider_status": signal.meta.get("provider_status") if isinstance(signal.meta, Mapping) else {},
                    "provider_latency_ms": signal.meta.get("provider_latency_ms") if isinstance(signal.meta, Mapping) else {},
                    "provider_selected": _provider_selected(signal),
                    "fallback_applied": _fallback_applied(signal),
                    "provider_exclusions": _provider_exclusions(signal),
                    "stale_domains": _stale_domains(signal),
                    "freshness_exclusions": _freshness_exclusions(signal),
                    "fast_vs_structural": _fast_vs_structural(signal),
                    "fast_structural_divergence": _fast_structural_divergence(signal),
                    "structural_confidence_score": _meta_scalar(signal, "structural_confidence_score"),
                    "fast_pressure_score": _meta_scalar(signal, "fast_pressure_score"),
                    "fast_structural_alignment": _meta_scalar(signal, "fast_structural_alignment"),
                    "fast_structural_divergence_score": _meta_scalar(signal, "fast_structural_divergence_score"),
                    "horizon_conflict": _meta_scalar(signal, "horizon_conflict"),
                    "cross_horizon_alignment": _meta_scalar(signal, "cross_horizon_alignment"),
                    "domains_excluded_by_freshness": _freshness_exclusions(signal),
                    "domains_with_fallback_active": _domains_with_fallback(signal),
                    "providers_circuit_open": _providers_circuit_open(signal),
                    "snapshot_classification": _meta_scalar(signal, "snapshot_classification"),
                },
            )
            self.events.append(event)
            recorded.append(event)
        return tuple(recorded)


def _primary_scenario_name(batch: RadarSignalBatch, signal: RadarSignal) -> str:
    scenarios = batch.scenarios_by_timeframe.get(signal.timeframe, ())
    if not scenarios:
        return "no_scenario"
    return scenarios[0].name


def _meta_list(meta: Mapping[str, Any], key: str) -> tuple[str, ...]:
    raw = meta.get(key) if isinstance(meta, Mapping) else None
    if isinstance(raw, (list, tuple)):
        return tuple(str(item) for item in raw)
    return ()


def _fast_vs_structural(signal: RadarSignal) -> str:
    if not isinstance(signal.meta, Mapping):
        return "unknown"
    horizon = signal.meta.get("horizon_scores")
    if not isinstance(horizon, Mapping):
        return "unknown"
    intraday = horizon.get("intraday")
    positional = horizon.get("positional")
    if not isinstance(intraday, (int, float)) or not isinstance(positional, (int, float)):
        return "unknown"
    if intraday >= positional + 8:
        return "fast_signal"
    if positional >= intraday + 8:
        return "structural_signal"
    return "balanced"


def _stale_domains(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    freshness = signal.meta.get("freshness")
    if not isinstance(freshness, Mapping):
        return ()
    stale: list[str] = []
    for horizon_data in freshness.values():
        if not isinstance(horizon_data, Mapping):
            continue
        for domain, payload in horizon_data.items():
            if not isinstance(payload, Mapping):
                continue
            if payload.get("status") == "stale":
                stale.append(str(domain))
    return tuple(sorted(set(stale)))


def _fast_structural_divergence(signal: RadarSignal) -> float | None:
    if not isinstance(signal.meta, Mapping):
        return None
    horizon = signal.meta.get("horizon_scores")
    if not isinstance(horizon, Mapping):
        return None
    intraday = horizon.get("intraday")
    positional = horizon.get("positional")
    if not isinstance(intraday, (int, float)) or not isinstance(positional, (int, float)):
        return None
    return float(intraday) - float(positional)


def _freshness_exclusions(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    freshness = signal.meta.get("freshness")
    if not isinstance(freshness, Mapping):
        return ()
    excluded: list[str] = []
    for horizon, horizon_data in freshness.items():
        if not isinstance(horizon_data, Mapping):
            continue
        for domain, payload in horizon_data.items():
            if not isinstance(payload, Mapping):
                continue
            status = payload.get("status")
            if status in {"stale", "non_operable"}:
                excluded.append(f"{horizon}:{domain}")
    return tuple(sorted(set(excluded)))


def _provider_selected(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    status = signal.meta.get("provider_status")
    if not isinstance(status, Mapping):
        return ()
    return tuple(sorted(str(key) for key in status.keys()))


def _fallback_applied(signal: RadarSignal) -> bool:
    if not isinstance(signal.meta, Mapping):
        return False
    status = signal.meta.get("provider_status")
    if not isinstance(status, Mapping):
        return False
    return any("fallback" in str(key).lower() for key in status.keys())


def _provider_exclusions(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    status = signal.meta.get("provider_status")
    if not isinstance(status, Mapping):
        return ()
    excluded = [
        str(key)
        for key, value in status.items()
        if str(value).lower() in {"error", "degraded", "empty", "unknown"}
    ]
    return tuple(sorted(excluded))


def _meta_scalar(signal: RadarSignal, key: str) -> Any:
    if not isinstance(signal.meta, Mapping):
        return None
    return signal.meta.get(key)


def _domains_with_fallback(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    status = signal.meta.get("provider_status")
    if not isinstance(status, Mapping):
        return ()
    out = [str(k) for k, v in status.items() if "degraded" in str(v).lower() or "empty" in str(v).lower()]
    return tuple(sorted(set(out)))


def _providers_circuit_open(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, Mapping):
        return ()
    provider_status = signal.meta.get("provider_status")
    if not isinstance(provider_status, Mapping):
        return ()
    open_keys = [str(k) for k, v in provider_status.items() if str(v).lower() == "error"]
    return tuple(sorted(set(open_keys)))
