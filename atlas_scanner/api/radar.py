from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import asyncio
import os
from pathlib import Path
from typing import Any, Mapping

from atlas_scanner.contracts import RadarDecisionHandoff, RadarSignalBatch, RadarTimeframe
from atlas_scanner.decision import (
    DecisionGateEvaluation,
    JsonlDecisionGateStore,
    config_from_env as decision_gate_config_from_env,
    evaluate_handoff,
)
from atlas_scanner.inference import (
    HandoffRegistry,
    NoOpHandoffConsumer,
    RadarRunInput,
    build_radar_handoff,
    run_radar_first_cut,
)
from atlas_scanner.inference.http_handoff_consumer import (
    HttpHandoffConsumer,
    config_from_env as handoff_http_config_from_env,
)
from atlas_scanner.perception.market import (
    OpenBBRealtimeAdapter,
    build_gamma_context,
    resolve_flow_provider,
    resolve_options_chain_provider,
    normalize_flow_events,
)
from atlas_scanner.perception.macro import resolve_macro_provider
from atlas_scanner.perception.institutional import (
    resolve_insider_provider,
    resolve_institutional_ownership_provider,
)
from atlas_scanner.perception.political import resolve_political_provider
from atlas_scanner.perception.regulatory import resolve_regulatory_provider
from atlas_scanner.runner.provider_resolution import RealtimeProviderRegistry
from atlas_scanner.store import JsonlRadarSnapshotStore
from atlas_scanner.telemetry import InMemoryRadarTelemetryRecorder, RadarTelemetryEvent
from atlas_scanner.ui.events import event_bus_from_env, to_sse_frame
from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import StreamingResponse

router = APIRouter(prefix="/api/radar", tags=["Radar"])
_PROVIDER_REGISTRY = RealtimeProviderRegistry()
_REALTIME_ADAPTER = OpenBBRealtimeAdapter()
_HANDOFF_REGISTRY = HandoffRegistry()
_HANDOFF_REGISTRY.register("noop", NoOpHandoffConsumer())
_HTTP_HANDOFF_CONFIG = handoff_http_config_from_env()
if _HTTP_HANDOFF_CONFIG is not None:
    _HANDOFF_REGISTRY.register("http", HttpHandoffConsumer(config=_HTTP_HANDOFF_CONFIG))
_STORE_PATH = Path(os.getenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", "data/atlas_scanner/radar_snapshots.jsonl"))
_STORE_MAX_RECORDS = int(os.getenv("ATLAS_SCANNER_RADAR_STORE_MAX_RECORDS", "5000"))
_STORE_MAX_BYTES = int(os.getenv("ATLAS_SCANNER_RADAR_STORE_MAX_BYTES", "8000000"))
_SNAPSHOT_PERSISTENCE = JsonlRadarSnapshotStore(
    _STORE_PATH,
    max_records=_STORE_MAX_RECORDS,
    max_bytes=_STORE_MAX_BYTES,
)
_SNAPSHOT_STORE: dict[str, dict[str, Any]] = {}
_PAPER_SUPERVISED_QUEUE: list[dict[str, Any]] = []
_DECISION_STORE_PATH = Path(os.getenv("ATLAS_DECISION_GATE_STORE_PATH", "data/atlas_scanner/decision_gate.jsonl"))
_DECISION_STORE_MAX_RECORDS = int(os.getenv("ATLAS_DECISION_GATE_STORE_MAX_RECORDS", "12000"))
_DECISION_STORE_MAX_BYTES = int(os.getenv("ATLAS_DECISION_GATE_STORE_MAX_BYTES", "8000000"))
_DECISION_STORE = JsonlDecisionGateStore(
    _DECISION_STORE_PATH,
    max_records=_DECISION_STORE_MAX_RECORDS,
    max_bytes=_DECISION_STORE_MAX_BYTES,
)
_RADAR_STREAM_ENABLED = os.getenv("ATLAS_ENABLE_RADAR_STREAM", "false").strip().lower() in {"1", "true", "yes"}
_RADAR_STREAM_MODE = os.getenv("ATLAS_RADAR_STREAM_MODE", "sse").strip().lower() or "sse"
_RADAR_STREAM_HEARTBEAT_SEC = max(3, int(os.getenv("ATLAS_RADAR_STREAM_HEARTBEAT_SEC", "10")))
_RADAR_STREAM_BUFFER_SIZE = max(100, int(os.getenv("ATLAS_RADAR_STREAM_BUFFER_SIZE", "1024")))
_RADAR_BACKPLANE_TYPE = os.getenv("ATLAS_RADAR_BACKPLANE_TYPE", "memory").strip().lower() or "memory"
_RADAR_REDIS_URL = os.getenv("ATLAS_RADAR_REDIS_URL", "").strip()
_RADAR_REDIS_CHANNEL = os.getenv("ATLAS_RADAR_REDIS_CHANNEL", "atlas:radar:stream").strip() or "atlas:radar:stream"
_RADAR_BACKPLANE_BUFFER_SIZE = max(100, int(os.getenv("ATLAS_RADAR_BACKPLANE_BUFFER_SIZE", "1024")))
_STREAM_BUS = event_bus_from_env()
_LAST_PROVIDER_STREAM_STATE: dict[str, tuple[bool, bool, str | None]] = {}
_RADAR_V4_INTEGRATION_MODE = os.getenv("ATLAS_RADAR_V4_INTEGRATION_MODE", "false").strip().lower() in {"1", "true", "yes"}
_RADAR_V4_AUTH_ENABLED = os.getenv("ATLAS_RADAR_V4_AUTH_ENABLED", "false").strip().lower() in {"1", "true", "yes"}
_RADAR_V4_AUTH_TOKEN = os.getenv("ATLAS_RADAR_V4_AUTH_TOKEN", "").strip()
_RADAR_SUPPORTED_TIMEFRAMES: tuple[str, ...] = ("1m", "5m", "15m", "1h", "1d")
_RADAR_ACTIVE_DOMAINS: tuple[str, ...] = (
    "market",
    "flow",
    "dealer",
    "macro",
    "ownership",
    "insider",
    "political",
    "regulatory",
    "gamma",
)


@dataclass(frozen=True)
class RadarApiResult:
    batch: RadarSignalBatch
    handoff: RadarDecisionHandoff
    telemetry: tuple[RadarTelemetryEvent, ...]


def run_institutional_radar(
    *,
    symbol: str,
    market_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]],
    flow_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]],
    runtime_mode: str = "paper",
    market_session: str = "regular",
    provider_status: Mapping[str, str] | None = None,
    provider_latency_ms: Mapping[str, int] | None = None,
    timeframes: tuple[RadarTimeframe, ...] | None = None,
    macro_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]] | None = None,
    institutional_payload: Mapping[str, Any] | None = None,
    insider_payload: Mapping[str, Any] | None = None,
    political_payload: Mapping[str, Any] | None = None,
    regulatory_payload: Mapping[str, Any] | None = None,
    telemetry_recorder: InMemoryRadarTelemetryRecorder | None = None,
) -> RadarApiResult:
    batch = run_radar_first_cut(
        RadarRunInput(
            symbol=symbol,
            market_by_timeframe=market_by_timeframe,
            flow_by_timeframe=flow_by_timeframe,
            runtime_mode=runtime_mode,
            market_session=market_session,
            provider_status=provider_status,
            provider_latency_ms=provider_latency_ms,
            timeframes=timeframes or tuple(market_by_timeframe.keys()),
            macro_by_timeframe=macro_by_timeframe,
            institutional_payload=institutional_payload,
            insider_payload=insider_payload,
            political_payload=political_payload,
            regulatory_payload=regulatory_payload,
        )
    )
    handoff = build_radar_handoff(batch)
    gate_config = decision_gate_config_from_env()
    gate_eval = evaluate_handoff(handoff, gate_config)
    _DECISION_STORE.append(gate_eval)
    _publish_decision_event(gate_eval)
    handoff = RadarDecisionHandoff(
        symbol=handoff.symbol,
        as_of=handoff.as_of,
        operable=handoff.operable,
        primary_timeframe=handoff.primary_timeframe,
        primary_signal=handoff.primary_signal,
        primary_scenarios=handoff.primary_scenarios,
        signals=handoff.signals,
        degradation_reasons=handoff.degradation_reasons,
        handoff_summary=handoff.handoff_summary,
        metadata={
            **dict(handoff.metadata),
            "decision_gate": {
                "enabled": gate_config.enabled,
                "mode": gate_config.mode,
                "decision": gate_eval.decision,
                "reason": gate_eval.reason,
                "snapshot_classification": gate_eval.snapshot_classification,
            },
        },
    )
    recorder = telemetry_recorder or InMemoryRadarTelemetryRecorder()
    events = recorder.record_batch(
        batch,
        pipeline="institutional_radar_first_cut",
        sources_used=tuple((provider_status or {}).keys()),
    )
    if gate_config.enabled and gate_config.mode == "paper_supervised":
        _PAPER_SUPERVISED_QUEUE.append(
            {
                "symbol": handoff.symbol,
                "as_of": handoff.as_of.isoformat(),
                "decision": gate_eval.decision,
                "reason": gate_eval.reason,
                "snapshot_classification": gate_eval.snapshot_classification,
                "summary": handoff.handoff_summary,
            }
        )
        if len(_PAPER_SUPERVISED_QUEUE) > 2000:
            del _PAPER_SUPERVISED_QUEUE[: len(_PAPER_SUPERVISED_QUEUE) - 2000]
        delivered = ("paper_supervised_queue",)
    else:
        delivered = _HANDOFF_REGISTRY.publish(handoff)
    _ = delivered
    return RadarApiResult(batch=batch, handoff=handoff, telemetry=events)


def build_realtime_snapshot(
    *,
    symbol: str,
    timeframes: tuple[RadarTimeframe, ...],
    runtime_mode: str = "paper",
    use_real_flow_provider: bool = False,
) -> RadarApiResult:
    market_by_timeframe: dict[RadarTimeframe, dict[str, Any]] = {}
    flow_by_timeframe: dict[RadarTimeframe, dict[str, Any]] = {}
    macro_by_timeframe: dict[RadarTimeframe, dict[str, Any]] = {}
    provider_status: dict[str, str] = {}
    provider_latency_ms: dict[str, int] = {}
    resolution = resolve_flow_provider(
        use_real_provider=use_real_flow_provider,
        provider_name=os.getenv("ATLAS_FLOW_PROVIDER"),
        fallback_name=os.getenv("ATLAS_FLOW_PROVIDER_FALLBACK"),
    )
    flow_provider = resolution.provider
    options_chain_provider = resolve_options_chain_provider()
    macro_provider = resolve_macro_provider()
    ownership_provider = resolve_institutional_ownership_provider()
    insider_provider = resolve_insider_provider()
    political_provider = resolve_political_provider()
    regulatory_provider = resolve_regulatory_provider()
    now = datetime.now(timezone.utc)
    flow_events = flow_provider.fetch_events(
        symbol=symbol,
        since=now - timedelta(days=1),
        until=now,
    )
    if resolution.provider_name == "unusual_whales" and not flow_events:
        fallback_resolution = resolve_flow_provider(
            provider_name=os.getenv("ATLAS_FLOW_PROVIDER_FALLBACK", "synthetic"),
            fallback_name="synthetic",
        )
        flow_events = fallback_resolution.provider.fetch_events(
            symbol=symbol,
            since=now - timedelta(days=1),
            until=now,
        )
        resolution = fallback_resolution
    flow_provider_diag = getattr(flow_provider, "last_diagnostics", {})
    ownership_snapshot = ownership_provider.fetch(symbol=symbol, as_of=now)
    ownership_diag = getattr(ownership_provider, "last_diagnostics", {})
    insider_snapshot = insider_provider.fetch(symbol=symbol, as_of=now)
    insider_diag = getattr(insider_provider, "last_diagnostics", {})
    political_snapshot = political_provider.fetch(scope=symbol, as_of=now)
    political_diag = getattr(political_provider, "last_diagnostics", {})
    regulatory_snapshot = regulatory_provider.fetch(symbol_or_scope=symbol, as_of=now)
    regulatory_diag = getattr(regulatory_provider, "last_diagnostics", {})

    macro_snapshot = macro_provider.fetch(scope=symbol, as_of=now)
    for timeframe in timeframes:
        preferred_provider = _PROVIDER_REGISTRY.resolve_provider(
            source_type="market", preferred=_REALTIME_ADAPTER.providers_chain
        )
        fetch = _REALTIME_ADAPTER.fetch(symbol=symbol, timeframe=timeframe)
        provider_status[f"market:{timeframe}"] = fetch.snapshot.diagnostics.get("provider_status", "unknown")
        provider_latency_ms[f"market:{timeframe}"] = fetch.latency_ms
        _PROVIDER_REGISTRY.record(
            source_type="market",
            provider_name=fetch.provider_used if fetch.provider_used != "none" else preferred_provider,
            status=provider_status[f"market:{timeframe}"],
            latency_ms=fetch.latency_ms,
            error=None if provider_status[f"market:{timeframe}"] == "ok" else "missing_market_data",
            extras={
                "provider_ready": provider_status[f"market:{timeframe}"] == "ok",
                "availability": provider_status[f"market:{timeframe}"],
            },
        )
        market_by_timeframe[timeframe] = {
            "spot_price": fetch.snapshot.spot_price,
            "close_price": fetch.snapshot.close_price,
            "return_pct": fetch.snapshot.return_pct,
            "momentum_score": fetch.snapshot.momentum_score,
            "relative_volume": fetch.snapshot.relative_volume,
            "volume_acceleration": fetch.snapshot.volume_acceleration,
            "meta": fetch.snapshot.meta,
        }
        flow_snapshot = normalize_flow_events(
            symbol=symbol,
            timeframe=timeframe,
            events=flow_events,
            as_of=now,
            diagnostics={
                "provider_status": "vendor" if use_real_flow_provider else "synthetic",
                "flow_provider_mode": resolution.provider_name,
                "flow_provider_fallback_used": str(resolution.fallback_used).lower(),
                "flow_provider_reason": resolution.reason or "",
                "flow_provider_diag_status": str(flow_provider_diag.get("status", "")),
            },
        )
        chain_payload = options_chain_provider.fetch_chain(symbol=symbol, as_of=now)
        options_chain_diag = getattr(options_chain_provider, "last_diagnostics", {})
        gamma_context = build_gamma_context(
            symbol=symbol,
            as_of=now,
            spot_price=fetch.snapshot.spot_price,
            options_chain=chain_payload,
            flow_snapshot=flow_snapshot,
        )
        provider_status[f"flow:{timeframe}"] = "ok" if flow_snapshot.quality_flags.get("has_flow") else "empty"
        provider_status[f"gamma:{timeframe}"] = "degraded" if gamma_context.degraded else "ok"
        provider_status[f"options_chain:{timeframe}"] = (
            "ok" if not gamma_context.degraded else "degraded"
        )
        provider_latency_ms[f"flow:{timeframe}"] = 0
        provider_latency_ms[f"gamma:{timeframe}"] = 0
        provider_latency_ms[f"options_chain:{timeframe}"] = int((options_chain_diag or {}).get("latency_ms", 0) or 0)
        _PROVIDER_REGISTRY.record(
            source_type="flow",
            provider_name=resolution.provider_name,
            status="ok" if flow_snapshot.quality_flags.get("has_flow") else "empty",
            latency_ms=0,
            error=resolution.reason if resolution.fallback_used else None,
            extras={
                "fallback_active": bool(resolution.fallback_used),
                "provider_ready": bool(flow_snapshot.quality_flags.get("has_flow", False)),
                "availability": "ready" if flow_snapshot.quality_flags.get("has_flow") else "empty",
                "circuit_state": str((flow_provider_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(flow_provider_diag or {})},
            },
        )
        _PROVIDER_REGISTRY.record(
            source_type="gamma",
            provider_name="options_chain" if not gamma_context.degraded else "proxy",
            status="empty" if gamma_context.degraded else "ok",
            latency_ms=0,
            error="degraded_proxy" if gamma_context.degraded else None,
            extras={
                "provider_ready": not gamma_context.degraded,
                "availability": "degraded" if gamma_context.degraded else "ready",
                "details": {"gamma_meta": dict(gamma_context.meta)},
            },
        )
        _PROVIDER_REGISTRY.record(
            source_type="options_chain",
            provider_name=str((options_chain_diag or {}).get("provider", "options_chain_stub")),
            status=str((options_chain_diag or {}).get("status", "degraded")),
            latency_ms=provider_latency_ms[f"options_chain:{timeframe}"],
            error=(options_chain_diag or {}).get("error"),
            extras={
                "fallback_active": bool((options_chain_diag or {}).get("fallback_active", True)),
                "provider_ready": bool((options_chain_diag or {}).get("provider_ready", False)),
                "availability": "ready" if bool((options_chain_diag or {}).get("provider_ready", False)) else "degraded",
                "circuit_state": str((options_chain_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(options_chain_diag or {})},
            },
        )
        flow_by_timeframe[timeframe] = {
            "equity_flow_bias": flow_snapshot.equity_flow_bias,
            "call_volume": flow_snapshot.call_volume,
            "put_volume": flow_snapshot.put_volume,
            "call_premium": flow_snapshot.call_premium,
            "put_premium": flow_snapshot.put_premium,
            "net_bias": flow_snapshot.net_bias,
            "aggression_score": flow_snapshot.aggression_score,
            "dte_distribution": dict(flow_snapshot.dte_distribution),
            "dte_call_premium": dict(flow_snapshot.dte_call_premium),
            "dte_put_premium": dict(flow_snapshot.dte_put_premium),
            "net_gamma": gamma_context.gamma_flip_level,
            "oi_concentration": None,
            "iv_context": None,
            "quality_flags": dict(flow_snapshot.quality_flags),
            "meta": {
                "gamma_context_confidence": gamma_context.confidence,
                "gamma_degradation_reasons": gamma_context.degradation_reasons,
                "dealer_pressure_score": gamma_context.meta.get("dealer_pressure_score"),
                "gamma_flip_confidence": gamma_context.meta.get("gamma_flip_confidence"),
            },
            "options_chain": {
                "gamma_by_strike": dict(chain_payload.get("gamma_by_strike", {})),
                "oi_by_strike": dict(chain_payload.get("oi_by_strike", {})),
                "call_oi_by_strike": dict(chain_payload.get("call_oi_by_strike", {})),
                "put_oi_by_strike": dict(chain_payload.get("put_oi_by_strike", {})),
                "dte_weights": dict(chain_payload.get("dte_weights", {})),
            },
        }
        macro_by_timeframe[timeframe] = {
            "upcoming_event": macro_snapshot.upcoming_event,
            "recent_event": macro_snapshot.recent_event,
            "high_impact_flag": macro_snapshot.high_impact_flag,
            "sector_sensitivity": macro_snapshot.sector_sensitivity,
            "confidence": macro_snapshot.confidence,
            "provider_ready": macro_snapshot.quality_flags.get("provider_ready", False),
            "upcoming_events": macro_snapshot.upcoming_events,
            "recent_events": macro_snapshot.recent_events,
            "calendar_risk_score": macro_snapshot.calendar_risk_score,
            "calendar_volatility_window": macro_snapshot.calendar_volatility_window,
        }
        provider_status[f"macro:{timeframe}"] = (
            "ok" if macro_snapshot.quality_flags.get("provider_ready", False) else "degraded"
        )
        provider_status[f"insider:{timeframe}"] = (
            "ok" if insider_snapshot.quality_flags.get("provider_ready", False) else "degraded"
        )
        provider_status[f"ownership:{timeframe}"] = (
            "ok" if ownership_snapshot.quality_flags.get("provider_ready", False) else "degraded"
        )
        provider_status[f"political:{timeframe}"] = (
            "ok" if political_snapshot.quality_flags.get("provider_ready", False) else "degraded"
        )
        provider_status[f"regulatory:{timeframe}"] = (
            "ok" if regulatory_snapshot.quality_flags.get("provider_ready", False) else "degraded"
        )
        provider_latency_ms[f"macro:{timeframe}"] = 0
        provider_latency_ms[f"ownership:{timeframe}"] = int((ownership_diag or {}).get("latency_ms", 0) or 0)
        provider_latency_ms[f"insider:{timeframe}"] = int((insider_diag or {}).get("latency_ms", 0) or 0)
        provider_latency_ms[f"political:{timeframe}"] = int((political_diag or {}).get("latency_ms", 0) or 0)
        provider_latency_ms[f"regulatory:{timeframe}"] = int((regulatory_diag or {}).get("latency_ms", 0) or 0)
        macro_diag = getattr(macro_provider, "last_diagnostics", {})
        _PROVIDER_REGISTRY.record(
            source_type="macro",
            provider_name=macro_snapshot.source,
            status=provider_status[f"macro:{timeframe}"],
            latency_ms=int((macro_diag or {}).get("latency_ms", 0) or 0),
            error=(macro_diag or {}).get("error"),
            extras={
                "fallback_active": bool((macro_diag or {}).get("fallback_active", False)),
                "provider_ready": bool(macro_snapshot.quality_flags.get("provider_ready", False)),
                "availability": "ready" if macro_snapshot.quality_flags.get("provider_ready", False) else "degraded",
                "circuit_state": str((macro_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(macro_diag or {})},
            },
        )
        calendar_diag = (macro_diag or {}).get("calendar_diag", {})
        if isinstance(calendar_diag, Mapping) and calendar_diag:
            _PROVIDER_REGISTRY.record(
                source_type="calendar",
                provider_name=str(calendar_diag.get("provider", "calendar")),
                status=str(calendar_diag.get("status", "unknown")),
                latency_ms=int(calendar_diag.get("latency_ms", 0) or 0),
                error=calendar_diag.get("error"),
                extras={
                    "provider_ready": str(calendar_diag.get("status", "")) in {"ok", "empty"},
                    "fallback_active": bool((macro_diag or {}).get("fallback_active", False)),
                    "circuit_state": str((calendar_diag.get("circuit_breaker", {}) or {}).get("state", "")) or None,
                    "details": {"diag": dict(calendar_diag)},
                },
            )
        _PROVIDER_REGISTRY.record(
            source_type="ownership",
            provider_name=ownership_snapshot.source,
            status=provider_status[f"ownership:{timeframe}"],
            latency_ms=provider_latency_ms[f"ownership:{timeframe}"],
            error=(ownership_diag or {}).get("error"),
            extras={
                "fallback_active": bool((ownership_diag or {}).get("fallback_active", False)),
                "provider_ready": bool(ownership_snapshot.quality_flags.get("provider_ready", False)),
                "availability": "ready" if ownership_snapshot.quality_flags.get("provider_ready", False) else "degraded",
                "circuit_state": str((ownership_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(ownership_diag or {}), "meta": dict(ownership_snapshot.meta)},
            },
        )
        _PROVIDER_REGISTRY.record(
            source_type="insider",
            provider_name=insider_snapshot.source,
            status=provider_status[f"insider:{timeframe}"],
            latency_ms=provider_latency_ms[f"insider:{timeframe}"],
            error=(insider_diag or {}).get("error"),
            extras={
                "fallback_active": bool((insider_diag or {}).get("fallback_active", False)),
                "provider_ready": bool(insider_snapshot.quality_flags.get("provider_ready", False)),
                "availability": "ready" if insider_snapshot.quality_flags.get("provider_ready", False) else "degraded",
                "circuit_state": str((insider_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(insider_diag or {}), "meta": dict(insider_snapshot.meta)},
            },
        )
        _PROVIDER_REGISTRY.record(
            source_type="political",
            provider_name=political_snapshot.source,
            status=provider_status[f"political:{timeframe}"],
            latency_ms=provider_latency_ms[f"political:{timeframe}"],
            error=(political_diag or {}).get("error"),
            extras={
                "fallback_active": bool((political_diag or {}).get("fallback_active", False)),
                "provider_ready": bool(political_snapshot.quality_flags.get("provider_ready", False)),
                "availability": "ready" if political_snapshot.quality_flags.get("provider_ready", False) else "degraded",
                "circuit_state": str((political_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(political_diag or {}), "meta": dict(political_snapshot.meta)},
            },
        )
        _PROVIDER_REGISTRY.record(
            source_type="regulatory",
            provider_name=regulatory_snapshot.source,
            status=provider_status[f"regulatory:{timeframe}"],
            latency_ms=provider_latency_ms[f"regulatory:{timeframe}"],
            error=(regulatory_diag or {}).get("error"),
            extras={
                "fallback_active": bool((regulatory_diag or {}).get("fallback_active", False)),
                "provider_ready": bool(regulatory_snapshot.quality_flags.get("provider_ready", False)),
                "availability": "ready" if regulatory_snapshot.quality_flags.get("provider_ready", False) else "degraded",
                "circuit_state": str((regulatory_diag or {}).get("circuit_breaker", {}).get("state", "")) or None,
                "details": {"diag": dict(regulatory_diag or {}), "meta": dict(regulatory_snapshot.meta)},
            },
        )

    result = run_institutional_radar(
        symbol=symbol,
        market_by_timeframe=market_by_timeframe,
        flow_by_timeframe=flow_by_timeframe,
        runtime_mode=runtime_mode,
        market_session="regular",
        provider_status=provider_status,
        provider_latency_ms=provider_latency_ms,
        timeframes=timeframes,
        macro_by_timeframe=macro_by_timeframe,
        institutional_payload={
            "ownership_pct": ownership_snapshot.ownership_pct,
            "ownership_delta_pct": ownership_snapshot.ownership_delta_pct,
            "concentration_score": ownership_snapshot.concentration_score,
            "delay_sec": ownership_snapshot.delay_sec,
            "freshness_sec": ownership_snapshot.freshness_sec,
            "confidence": ownership_snapshot.confidence,
            "provider_ready": ownership_snapshot.quality_flags.get("provider_ready", False),
            "top_holders": ownership_snapshot.meta.get("top_holders"),
            "holder_change_balance": ownership_snapshot.meta.get("holder_change_balance"),
            "sponsorship_score": ownership_snapshot.meta.get("sponsorship_score"),
            "ownership_signal": ownership_snapshot.meta.get("ownership_signal"),
            "delay_days": ownership_snapshot.meta.get("delay_days"),
        },
        insider_payload={
            "buy_sell_ratio": insider_snapshot.buy_sell_ratio,
            "net_insider_value": insider_snapshot.net_insider_value,
            "notable_buyers": insider_snapshot.notable_buyers,
            "notable_sellers": insider_snapshot.notable_sellers,
            "delay_sec": insider_snapshot.delay_sec,
            "freshness_sec": insider_snapshot.freshness_sec,
            "confidence": insider_snapshot.confidence,
            "provider_ready": insider_snapshot.quality_flags.get("provider_ready", False),
            "transaction_count": insider_snapshot.meta.get("transaction_count"),
            "net_signal": insider_snapshot.meta.get("net_signal"),
        },
        political_payload={
            "net_political_flow": political_snapshot.net_political_flow,
            "related_tickers": political_snapshot.related_tickers,
            "disclosure_lag_days": political_snapshot.disclosure_lag_days,
            "delay_sec": political_snapshot.delay_sec,
            "freshness_sec": political_snapshot.freshness_sec,
            "confidence": political_snapshot.confidence,
            "provider_ready": political_snapshot.quality_flags.get("provider_ready", False),
            "transaction_count": political_snapshot.meta.get("transaction_count"),
            "buy_count": political_snapshot.meta.get("buy_count"),
            "sell_count": political_snapshot.meta.get("sell_count"),
            "buy_sell_balance": political_snapshot.meta.get("buy_sell_balance"),
            "notable_entities": political_snapshot.meta.get("notable_entities"),
            "signal_strength": political_snapshot.meta.get("signal_strength"),
        },
        regulatory_payload={
            "event_type": regulatory_snapshot.event_type,
            "severity": regulatory_snapshot.severity,
            "overhang_score": regulatory_snapshot.overhang_score,
            "delay_sec": regulatory_snapshot.delay_sec,
            "freshness_sec": regulatory_snapshot.freshness_sec,
            "confidence": regulatory_snapshot.confidence,
            "provider_ready": regulatory_snapshot.quality_flags.get("provider_ready", False),
            "filing_count": regulatory_snapshot.meta.get("filing_count"),
            "latest_forms": regulatory_snapshot.meta.get("latest_forms"),
        },
    )
    _SNAPSHOT_PERSISTENCE.append_batch(
        batch=result.batch,
        provider_health=_PROVIDER_REGISTRY.snapshot(),
    )
    _SNAPSHOT_STORE[symbol.upper()] = {
        "at": datetime.now(timezone.utc).isoformat(),
        "result": result,
        "provider_health": _PROVIDER_REGISTRY.snapshot(),
    }
    _publish_snapshot_and_operability_events(result)
    _publish_provider_health_changes(_PROVIDER_REGISTRY.snapshot())
    return result


def build_macro_calendar_snapshot(*, symbol: str) -> dict[str, Any]:
    provider = resolve_macro_provider()
    now = datetime.now(timezone.utc)
    snapshot = provider.fetch(scope=symbol, as_of=now)
    return {
        "symbol": symbol.upper(),
        "as_of": now.isoformat(),
        "source": snapshot.source,
        "upcoming_events": [
            {
                "event_name": event.event_name,
                "release_datetime": event.release_datetime.isoformat(),
                "impact_level": event.impact_level,
                "affected_sectors": list(event.affected_sectors),
                "consensus": event.consensus,
                "previous_value": event.previous_value,
            }
            for event in snapshot.upcoming_events
        ],
        "recent_events": [
            {
                "event_name": event.event_name,
                "release_datetime": event.release_datetime.isoformat(),
                "actual_value": event.actual_value,
                "consensus": event.consensus,
                "surprise": event.surprise,
                "impact_level": event.impact_level,
                "affected_sectors": list(event.affected_sectors),
            }
            for event in snapshot.recent_events
        ],
        "calendar_risk_score": snapshot.calendar_risk_score,
        "calendar_volatility_window": snapshot.calendar_volatility_window,
        "affected_sectors": sorted(
            set(sector for event in snapshot.upcoming_events for sector in event.affected_sectors)
        ),
        "provider_ready": bool(snapshot.quality_flags.get("provider_ready", False)),
    }


@router.get("/health/providers")
async def get_provider_health() -> dict[str, Any]:
    """Retorna estado agregado de providers market/flow/gamma."""
    providers = _PROVIDER_REGISTRY.snapshot()
    return {
        "ok": True,
        "providers": providers,
        "observability": _provider_observability_summary(providers),
        "flow_provider_env": os.getenv("ATLAS_FLOW_PROVIDER", ""),
        "handoff_consumers": list(_HANDOFF_REGISTRY.consumers()),
    }


@router.get("/diagnostics/providers")
async def get_provider_diagnostics() -> dict[str, Any]:
    providers = _PROVIDER_REGISTRY.snapshot()
    return {
        "ok": True,
        "providers": providers,
        "summary": _provider_observability_summary(providers),
    }


@router.get("/stream")
async def radar_stream(
    request: Request,
    once: bool = Query(default=False),
) -> StreamingResponse:
    if not _RADAR_STREAM_ENABLED or _RADAR_STREAM_MODE != "sse":
        async def disabled() -> Any:
            yield "event: alert\ndata: {\"type\":\"alert\",\"payload\":{\"severity\":\"warning\",\"domain\":\"stream\",\"message\":\"stream_disabled\"}}\n\n"
        return StreamingResponse(disabled(), media_type="text/event-stream")

    last_id_raw = request.headers.get("last-event-id", "").strip()
    cursor = int(last_id_raw) if last_id_raw.isdigit() else _STREAM_BUS.latest_seq()

    async def event_generator() -> Any:
        nonlocal cursor
        while True:
            if await request.is_disconnected():
                break
            events = _STREAM_BUS.since(cursor)
            if events:
                for event in events:
                    cursor = event.seq
                    yield to_sse_frame(event)
            else:
                heartbeat = _STREAM_BUS.publish(
                    event_type="heartbeat",
                    payload={"status": "alive"},
                )
                cursor = heartbeat.seq
                yield to_sse_frame(heartbeat)
            if once:
                break
            await asyncio.sleep(_RADAR_STREAM_HEARTBEAT_SEC)

    headers = {
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",
    }
    return StreamingResponse(event_generator(), media_type="text/event-stream", headers=headers)


@router.get("/stream/metrics")
async def get_stream_metrics() -> dict[str, Any]:
    return {
        "ok": True,
        "stream_enabled": _RADAR_STREAM_ENABLED,
        "stream_mode": _RADAR_STREAM_MODE,
        "heartbeat_sec": _RADAR_STREAM_HEARTBEAT_SEC,
        "configured_backplane": _RADAR_BACKPLANE_TYPE,
        "redis_channel": _RADAR_REDIS_CHANNEL,
        "buffer_size": _RADAR_BACKPLANE_BUFFER_SIZE,
        "metrics": _STREAM_BUS.metrics(),
    }


@router.get("/snapshot/{symbol}")
async def get_radar_snapshot(
    symbol: str,
    timeframes: list[str] = Query(default=["1m", "5m", "15m"]),
    runtime_mode: str = Query(default="paper"),
    refresh: bool = Query(default=False),
    use_real_flow_provider: bool = Query(default=False),
) -> dict[str, Any]:
    """Retorna snapshot actual del radar por símbolo y temporalidades."""
    valid: tuple[RadarTimeframe, ...] = tuple(
        timeframe for timeframe in timeframes if timeframe in {"1m", "5m", "15m", "1h", "1d"}
    )  # type: ignore[assignment]
    if not valid:
        valid = ("1m", "5m", "15m")
    cache_key = symbol.upper()
    cached = _SNAPSHOT_STORE.get(cache_key)
    if cached and not refresh:
        result: RadarApiResult = cached["result"]
    else:
        result = build_realtime_snapshot(
            symbol=symbol,
            timeframes=valid,
            runtime_mode=runtime_mode,
            use_real_flow_provider=use_real_flow_provider,
        )
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "timeframes": list(valid),
        "handoff": {
            "operable": result.handoff.operable,
            "summary": result.handoff.handoff_summary,
            "primary_timeframe": result.handoff.primary_timeframe,
            "degradation_reasons": list(result.handoff.degradation_reasons),
            "decision_gate": dict(result.handoff.metadata.get("decision_gate", {})) if isinstance(result.handoff.metadata, Mapping) else {},
        },
        "signals": [
            {
                "timeframe": signal.timeframe,
                "aggregate_conviction_score": signal.aggregate_conviction_score,
                "primary_conviction_reason": signal.primary_conviction_reason,
                "primary_degradation_reason": signal.primary_degradation_reason,
                "operable": signal.quality.is_operable,
                "active_domains": signal.meta.get("active_domains", ()),
                "degraded_domains": signal.meta.get("degraded_domains", ()),
                "freshness": signal.meta.get("freshness", {}),
                "effective_weights": signal.meta.get("effective_weights", {}),
                "snapshot_classification": signal.meta.get("snapshot_classification"),
                "structural_confidence_score": signal.meta.get("structural_confidence_score"),
                "structural_bullish_score": signal.meta.get("structural_bullish_score"),
                "structural_bearish_score": signal.meta.get("structural_bearish_score"),
                "fast_pressure_score": signal.meta.get("fast_pressure_score"),
                "fast_directional_bias": signal.meta.get("fast_directional_bias"),
                "fast_risk_score": signal.meta.get("fast_risk_score"),
                "fast_structural_alignment": signal.meta.get("fast_structural_alignment"),
                "fast_structural_divergence_score": signal.meta.get("fast_structural_divergence_score"),
                "horizon_conflict": signal.meta.get("horizon_conflict"),
            }
            for signal in result.batch.signals
        ],
        "providers": _PROVIDER_REGISTRY.snapshot(),
    }


@router.get("/recent")
async def get_recent_snapshots(
    timeframe: str | None = Query(default=None),
    limit: int = Query(default=50, ge=1, le=500),
    only_operable: bool = Query(default=False),
    include_degraded: bool = Query(default=True),
) -> dict[str, Any]:
    rows = _SNAPSHOT_PERSISTENCE.recent(
        limit=limit,
        timeframe=timeframe,
        only_operable=only_operable,
        include_degraded=include_degraded,
    )
    return {
        "ok": True,
        "count": len(rows),
        "items": [
            {
                "symbol": row.symbol,
                "timeframe": row.timeframe,
                "timestamp": row.timestamp,
                "operable": row.operable,
                "aggregate_conviction_score": row.aggregate_conviction_score,
                "degradation_reasons": list(row.degradation_reasons),
            }
            for row in rows
        ],
    }


@router.get("/replay/{symbol}")
async def get_radar_replay(
    symbol: str,
    timeframe: str | None = Query(default=None),
    limit: int = Query(default=100, ge=1, le=1000),
    include_degraded: bool = Query(default=True),
) -> dict[str, Any]:
    rows = _SNAPSHOT_PERSISTENCE.replay(
        symbol=symbol,
        timeframe=timeframe,
        limit=limit,
        include_degraded=include_degraded,
    )
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "count": len(rows),
        "items": [
            {
                "timeframe": row.timeframe,
                "timestamp": row.timestamp,
                "operable": row.operable,
                "signal": row.signal,
                "scenarios": list(row.scenarios),
                "quality_flags": row.quality_flags,
                "provider_diagnostics": row.provider_diagnostics,
            }
            for row in rows
        ],
    }


@router.get("/store/stats")
async def get_store_stats() -> dict[str, Any]:
    return {
        "ok": True,
        "store": _SNAPSHOT_PERSISTENCE.stats(),
    }


@router.get("/diagnostics/freshness/{symbol}")
async def get_freshness_diagnostics(
    symbol: str,
    timeframes: list[str] = Query(default=["1m", "5m", "15m"]),
    runtime_mode: str = Query(default="paper"),
) -> dict[str, Any]:
    valid: tuple[RadarTimeframe, ...] = tuple(
        timeframe for timeframe in timeframes if timeframe in {"1m", "5m", "15m", "1h", "1d"}
    )  # type: ignore[assignment]
    if not valid:
        valid = ("1m", "5m", "15m")
    result = build_realtime_snapshot(
        symbol=symbol,
        timeframes=valid,
        runtime_mode=runtime_mode,
        use_real_flow_provider=False,
    )
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "items": [
            {
                "timeframe": signal.timeframe,
                "freshness": signal.meta.get("freshness", {}),
                "effective_weights": signal.meta.get("effective_weights", {}),
                "active_domains": signal.meta.get("active_domains", ()),
                "degraded_domains": signal.meta.get("degraded_domains", ()),
            }
            for signal in result.batch.signals
        ],
    }


@router.get("/diagnostics/structural/{symbol}")
async def get_structural_diagnostics(
    symbol: str,
    runtime_mode: str = Query(default="paper"),
) -> dict[str, Any]:
    result = build_realtime_snapshot(
        symbol=symbol,
        timeframes=("1h", "1d"),
        runtime_mode=runtime_mode,
        use_real_flow_provider=False,
    )
    primary = result.batch.primary_signal
    if primary is None:
        return {"ok": False, "symbol": symbol.upper(), "reason": "missing_primary_signal"}
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "snapshot_classification": primary.meta.get("snapshot_classification"),
        "structural_confidence_score": primary.meta.get("structural_confidence_score"),
        "structural_bullish_score": primary.meta.get("structural_bullish_score"),
        "structural_bearish_score": primary.meta.get("structural_bearish_score"),
        "institutional_context": primary.meta.get("institutional_context", {}),
        "political_context": primary.meta.get("political_context", {}),
        "freshness": primary.meta.get("freshness", {}).get("swing", {}),
        "provider_status": _filter_provider_status(primary.meta.get("provider_status", {}), ("ownership:", "insider:", "political:", "regulatory:")),
        "degradation_reasons": list(primary.quality.degradation_reasons),
    }


@router.get("/diagnostics/fast/{symbol}")
async def get_fast_diagnostics(
    symbol: str,
    runtime_mode: str = Query(default="paper"),
) -> dict[str, Any]:
    result = build_realtime_snapshot(
        symbol=symbol,
        timeframes=("1m", "5m", "15m"),
        runtime_mode=runtime_mode,
        use_real_flow_provider=False,
    )
    primary = result.batch.primary_signal
    if primary is None:
        return {"ok": False, "symbol": symbol.upper(), "reason": "missing_primary_signal"}
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "snapshot_classification": primary.meta.get("snapshot_classification"),
        "fast_pressure_score": primary.meta.get("fast_pressure_score"),
        "fast_directional_bias": primary.meta.get("fast_directional_bias"),
        "fast_risk_score": primary.meta.get("fast_risk_score"),
        "fast_structural_alignment": primary.meta.get("fast_structural_alignment"),
        "fast_structural_divergence_score": primary.meta.get("fast_structural_divergence_score"),
        "horizon_conflict": primary.meta.get("horizon_conflict"),
        "cross_horizon_alignment": primary.meta.get("cross_horizon_alignment"),
        "dealer_context": primary.meta.get("dealer_context", {}),
        "macro_calendar": primary.meta.get("macro_calendar", {}),
        "freshness": primary.meta.get("freshness", {}).get("intraday", {}),
        "provider_status": _filter_provider_status(primary.meta.get("provider_status", {}), ("market:", "flow:", "dealer:", "gamma:", "options_chain:", "macro:")),
        "degradation_reasons": list(primary.quality.degradation_reasons),
    }


@router.get("/dealer/{symbol}")
async def get_dealer_diagnostics(
    symbol: str,
    runtime_mode: str = Query(default="paper"),
) -> dict[str, Any]:
    result = build_realtime_snapshot(symbol=symbol, timeframes=("1m", "5m", "15m"), runtime_mode=runtime_mode)
    primary = result.batch.primary_signal
    if primary is None:
        return {"ok": False, "symbol": symbol.upper(), "reason": "missing_primary_signal"}
    freshness = primary.meta.get("freshness", {}) if isinstance(primary.meta, Mapping) else {}
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "dealer": primary.meta.get("dealer_context", {}),
        "fast_pressure_score": primary.meta.get("fast_pressure_score"),
        "freshness": freshness.get("intraday", {}).get("dealer", {}) if isinstance(freshness, Mapping) else {},
        "provider_status": _filter_provider_status(primary.meta.get("provider_status", {}), ("gamma:", "options_chain:")),
        "providers": _filter_provider_health(_PROVIDER_REGISTRY.snapshot(), ("gamma:", "options_chain:")),
        "degradation_reasons": list(primary.quality.degradation_reasons),
    }


@router.get("/political/{symbol}")
async def get_political_diagnostics(
    symbol: str,
    runtime_mode: str = Query(default="paper"),
) -> dict[str, Any]:
    result = build_realtime_snapshot(symbol=symbol, timeframes=("1h", "1d"), runtime_mode=runtime_mode)
    primary = result.batch.primary_signal
    if primary is None:
        return {"ok": False, "symbol": symbol.upper(), "reason": "missing_primary_signal"}
    freshness = primary.meta.get("freshness", {}) if isinstance(primary.meta, Mapping) else {}
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "political_context": primary.meta.get("political_context", {}),
        "structural_confidence_score": primary.meta.get("structural_confidence_score"),
        "freshness": freshness.get("swing", {}).get("political", {}) if isinstance(freshness, Mapping) else {},
        "provider_status": _filter_provider_status(primary.meta.get("provider_status", {}), ("political:",)),
        "providers": _filter_provider_health(_PROVIDER_REGISTRY.snapshot(), ("political:",)),
        "degradation_reasons": list(primary.quality.degradation_reasons),
    }


@router.get("/macro/calendar/{symbol}")
async def get_macro_calendar(symbol: str) -> dict[str, Any]:
    data = build_macro_calendar_snapshot(symbol=symbol)
    return {"ok": True, **data}


@router.get("/decisions/recent")
async def get_decisions_recent(
    limit: int = Query(default=100, ge=1, le=1000),
) -> dict[str, Any]:
    rows = _DECISION_STORE.recent(limit=limit)
    return {
        "ok": True,
        "count": len(rows),
        "items": [_decision_row(item) for item in rows],
    }


@router.get("/decisions/replay/{symbol}")
async def get_decisions_replay(
    symbol: str,
    limit: int = Query(default=500, ge=1, le=5000),
) -> dict[str, Any]:
    rows = _DECISION_STORE.replay(symbol=symbol, limit=limit)
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "count": len(rows),
        "items": [_decision_row(item) for item in rows],
    }


@router.get("/decisions/stats")
async def get_decisions_stats() -> dict[str, Any]:
    return {
        "ok": True,
        "stats": _DECISION_STORE.stats(),
    }


@router.get("/decisions/{symbol}")
async def get_decisions_by_symbol(
    symbol: str,
    limit: int = Query(default=100, ge=1, le=1000),
) -> dict[str, Any]:
    filtered = _DECISION_STORE.recent(limit=limit, symbol=symbol)
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "count": len(filtered),
        "items": [_decision_row(item) for item in filtered],
    }


@router.get("/dashboard/summary")
async def get_dashboard_summary(
    symbol: str = Query(default="SPY"),
    limit: int = Query(default=20, ge=5, le=100),
) -> dict[str, Any]:
    snapshot = build_realtime_snapshot(
        symbol=symbol,
        timeframes=("1m", "5m", "15m", "1h"),
        runtime_mode="paper",
        use_real_flow_provider=False,
    )
    providers = _PROVIDER_REGISTRY.snapshot()
    decision_stats = _DECISION_STORE.stats()
    recent_decisions = _DECISION_STORE.recent(limit=limit, symbol=symbol)
    recent_snapshots = _SNAPSHOT_PERSISTENCE.recent(limit=limit, symbol=symbol)
    class_breakdown: dict[str, int] = {}
    alerts: list[str] = []
    freshness_domains: dict[str, dict[str, Any]] = {}
    for signal in snapshot.batch.signals:
        cls = str(signal.meta.get("snapshot_classification", "unknown"))
        class_breakdown[cls] = class_breakdown.get(cls, 0) + 1
        if signal.primary_degradation_reason:
            alerts.append(f"{signal.timeframe}:{signal.primary_degradation_reason}")
        if isinstance(signal.meta.get("freshness"), Mapping):
            freshness_domains[signal.timeframe] = dict(signal.meta.get("freshness", {}))
    circuit_open = [name for name, payload in providers.items() if isinstance(payload, Mapping) and bool(payload.get("circuit_open_indicator", False))]
    return {
        "ok": True,
        "symbol": symbol.upper(),
        "provider_health_summary": _provider_observability_summary(providers),
        "snapshot_classification_breakdown": class_breakdown,
        "decision_gate_stats": decision_stats,
        "recent_signals": [
            {
                "timeframe": sig.timeframe,
                "aggregate_conviction_score": sig.aggregate_conviction_score,
                "classification": sig.meta.get("snapshot_classification"),
                "primary_reason": sig.primary_conviction_reason,
                "degradation_reason": sig.primary_degradation_reason,
            }
            for sig in snapshot.batch.signals[:limit]
        ],
        "active_alerts_or_degradations": sorted(set(alerts)),
        "freshness_status_by_domain": freshness_domains,
        "circuit_breaker_status": {"open_providers": circuit_open, "count": len(circuit_open)},
        "recent_decisions": [_decision_row(item) for item in recent_decisions],
        "recent_snapshots": [
            {
                "timestamp": row.timestamp,
                "timeframe": row.timeframe,
                "operable": row.operable,
                "aggregate_conviction_score": row.aggregate_conviction_score,
                "degradation_reasons": list(row.degradation_reasons),
            }
            for row in recent_snapshots
        ],
    }


@router.get("/v4/summary")
async def get_v4_summary(
    request: Request,
    symbol: str = Query(default="SPY"),
    top_n: int = Query(default=5, ge=1, le=20),
    decisions_limit: int = Query(default=20, ge=5, le=100),
) -> dict[str, Any]:
    _ensure_v4_mode()
    _enforce_v4_auth(request)
    snapshot = build_realtime_snapshot(
        symbol=symbol,
        timeframes=("1m", "5m", "15m", "1h"),
        runtime_mode="paper",
        use_real_flow_provider=False,
    )
    primary = snapshot.batch.primary_signal
    providers = _PROVIDER_REGISTRY.snapshot()
    decision_rows = _DECISION_STORE.recent(limit=decisions_limit, symbol=symbol)
    decision_payload = [_decision_row(item) for item in decision_rows]
    recent_signals = sorted(snapshot.batch.signals, key=lambda sig: float(sig.aggregate_conviction_score), reverse=True)
    top_signals = [
        {
            "symbol": sig.symbol,
            "timeframe": sig.timeframe,
            "aggregate_conviction_score": sig.aggregate_conviction_score,
            "primary_conviction_reason": sig.primary_conviction_reason,
            "primary_degradation_reason": sig.primary_degradation_reason,
            "snapshot_classification": sig.meta.get("snapshot_classification"),
            "fast_pressure_score": sig.meta.get("fast_pressure_score"),
            "structural_confidence_score": sig.meta.get("structural_confidence_score"),
            "horizon_conflict": sig.meta.get("horizon_conflict"),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        for sig in recent_signals[:top_n]
    ]
    degradations_active = sorted(
        {
            str(sig.primary_degradation_reason)
            for sig in snapshot.batch.signals
            if sig.primary_degradation_reason
        }
    )
    last_update = datetime.now(timezone.utc).isoformat()
    freshness = primary.meta.get("freshness", {}) if primary is not None and isinstance(primary.meta, Mapping) else {}
    provider_summary = _provider_observability_summary(providers)
    return {
        "ok": True,
        "integration_mode": "v4",
        "symbol": symbol.upper(),
        "radar_status": {
            "operable": bool(snapshot.handoff.operable),
            "snapshot_classification": primary.meta.get("snapshot_classification") if primary is not None else "unknown",
            "summary": snapshot.handoff.handoff_summary,
        },
        "top_signals": top_signals,
        "gate_recent_decisions": decision_payload,
        "provider_health_summary": provider_summary,
        "degradations_active": degradations_active,
        "structural_context_summary": {
            "structural_confidence_score": primary.meta.get("structural_confidence_score") if primary is not None else None,
            "structural_bullish_score": primary.meta.get("structural_bullish_score") if primary is not None else None,
            "structural_bearish_score": primary.meta.get("structural_bearish_score") if primary is not None else None,
        },
        "fast_context_summary": {
            "fast_pressure_score": primary.meta.get("fast_pressure_score") if primary is not None else None,
            "fast_directional_bias": primary.meta.get("fast_directional_bias") if primary is not None else None,
            "fast_structural_alignment": primary.meta.get("fast_structural_alignment") if primary is not None else None,
            "horizon_conflict": primary.meta.get("horizon_conflict") if primary is not None else None,
        },
        "freshness": freshness,
        "last_update": last_update,
        "stream_available": _RADAR_STREAM_ENABLED and _RADAR_STREAM_MODE == "sse",
    }


@router.get("/v4/config")
async def get_v4_config(request: Request) -> dict[str, Any]:
    _ensure_v4_mode()
    _enforce_v4_auth(request)
    providers = _PROVIDER_REGISTRY.snapshot()
    gate_config = decision_gate_config_from_env()
    return {
        "ok": True,
        "integration_mode": "v4",
        "domains_active": list(_RADAR_ACTIVE_DOMAINS),
        "providers_available": sorted(providers.keys()),
        "modes_supported": ["paper"],
        "timeframes_supported": list(_RADAR_SUPPORTED_TIMEFRAMES),
        "streaming": {
            "available": _RADAR_STREAM_ENABLED and _RADAR_STREAM_MODE == "sse",
            "mode": _RADAR_STREAM_MODE,
            "heartbeat_sec": _RADAR_STREAM_HEARTBEAT_SEC,
        },
        "decision_gate_thresholds": {
            "min_structural_confidence": gate_config.min_structural_confidence,
            "min_fast_pressure": gate_config.min_fast_pressure,
            "max_divergence_score": gate_config.max_divergence_score,
            "allow_degraded": gate_config.allow_degraded,
            "allow_structural_only": gate_config.allow_structural_only,
            "allow_fast_only": gate_config.allow_fast_only,
        },
        "decision_gate_mode": gate_config.mode,
    }


def _filter_provider_status(status: Any, prefixes: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(status, Mapping):
        return {}
    return {str(k): v for k, v in status.items() if any(str(k).startswith(prefix) for prefix in prefixes)}


def _filter_provider_health(providers: Mapping[str, Any], prefixes: tuple[str, ...]) -> dict[str, Any]:
    return {
        key: value
        for key, value in providers.items()
        if any(str(key).startswith(prefix) for prefix in prefixes)
    }


def _provider_observability_summary(providers: Mapping[str, Any]) -> dict[str, Any]:
    p95_values = []
    burst = False
    circuit_open = False
    fallback = False
    for payload in providers.values():
        if not isinstance(payload, Mapping):
            continue
        p95 = payload.get("p95_latency_ms")
        if isinstance(p95, (int, float)):
            p95_values.append(float(p95))
        burst = burst or bool(payload.get("burst_error_indicator", False))
        circuit_open = circuit_open or bool(payload.get("circuit_open_indicator", False))
        fallback = fallback or bool(payload.get("active_fallback_indicator", False))
    return {
        "providers_count": len(providers),
        "p95_latency_ms_max": round(max(p95_values), 2) if p95_values else 0.0,
        "burst_error_indicator": burst,
        "circuit_open_indicator": circuit_open,
        "active_fallback_indicator": fallback,
    }


def _decision_row(item: Any) -> dict[str, Any]:
    return {
        "symbol": item.symbol,
        "timeframe": item.timeframe,
        "timestamp": item.timestamp,
        "classification": item.snapshot_classification,
        "structural_confidence_score": item.structural_confidence_score,
        "fast_pressure_score": item.fast_pressure_score,
        "fast_structural_alignment": item.fast_structural_alignment,
        "fast_structural_divergence_score": item.fast_structural_divergence_score,
        "decision": item.decision,
        "reason": item.reason,
        "mode": item.mode,
        "horizon_conflict": item.horizon_conflict,
        "cross_horizon_alignment": item.cross_horizon_alignment,
        "thresholds": item.thresholds,
    }


def _ensure_v4_mode() -> None:
    if _RADAR_V4_INTEGRATION_MODE:
        return
    raise HTTPException(status_code=404, detail="radar_v4_integration_disabled")


def _enforce_v4_auth(request: Request) -> None:
    if not _RADAR_V4_AUTH_ENABLED:
        return
    auth_header = request.headers.get("authorization", "").strip()
    service_token = request.headers.get("x-atlas-service-token", "").strip()
    expected = _RADAR_V4_AUTH_TOKEN
    if not expected:
        raise HTTPException(status_code=401, detail="radar_v4_auth_misconfigured")
    bearer_ok = auth_header.lower().startswith("bearer ") and auth_header[7:].strip() == expected
    service_ok = service_token == expected
    if bearer_ok or service_ok:
        return
    raise HTTPException(status_code=401, detail="radar_v4_auth_invalid")


def _publish_snapshot_and_operability_events(result: RadarApiResult) -> None:
    primary = result.batch.primary_signal
    if primary is None:
        return
    payload = {
        "symbol": primary.symbol,
        "timeframe": primary.timeframe,
        "snapshot_classification": primary.meta.get("snapshot_classification"),
        "fast_pressure_score": primary.meta.get("fast_pressure_score"),
        "structural_confidence_score": primary.meta.get("structural_confidence_score"),
        "fast_structural_alignment": primary.meta.get("fast_structural_alignment"),
        "horizon_conflict": primary.meta.get("horizon_conflict"),
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    _STREAM_BUS.publish(event_type="snapshot", payload=payload)
    if primary.primary_degradation_reason:
        _STREAM_BUS.publish(
            event_type="degradation",
            payload={
                "severity": "warning",
                "domain": "snapshot",
                "message": primary.primary_degradation_reason,
                "symbol": primary.symbol,
                "timeframe": primary.timeframe,
            },
        )
    if bool(primary.meta.get("horizon_conflict", False)):
        _STREAM_BUS.publish(
            event_type="alert",
            payload={
                "severity": "warning",
                "domain": "horizon",
                "message": "Conflicto detectado entre horizonte fast y structural.",
                "symbol": primary.symbol,
                "timeframe": primary.timeframe,
            },
        )


def _publish_decision_event(gate_eval: DecisionGateEvaluation) -> None:
    _STREAM_BUS.publish(
        event_type="decision",
        payload={
            "symbol": gate_eval.symbol,
            "timeframe": gate_eval.timeframe,
            "decision": gate_eval.decision,
            "reason": gate_eval.reason,
            "classification": gate_eval.snapshot_classification,
            "timestamp": gate_eval.timestamp.astimezone(timezone.utc).isoformat(),
        },
    )
    if gate_eval.decision in {"rejected", "caution"}:
        _STREAM_BUS.publish(
            event_type="alert",
            payload={
                "severity": "warning" if gate_eval.decision == "caution" else "critical",
                "domain": "decision_gate",
                "message": f"{gate_eval.decision}: {gate_eval.reason}",
                "symbol": gate_eval.symbol,
                "timeframe": gate_eval.timeframe,
            },
        )


def _publish_provider_health_changes(providers: Mapping[str, Any]) -> None:
    for key, payload_any in providers.items():
        if not isinstance(payload_any, Mapping):
            continue
        provider_name = str(payload_any.get("provider_name") or key)
        provider_ready = bool(payload_any.get("provider_ready", False))
        fallback_active = bool(payload_any.get("active_fallback_indicator", payload_any.get("fallback_active", False)))
        circuit_state_raw = payload_any.get("circuit_state")
        circuit_state = str(circuit_state_raw) if circuit_state_raw is not None else None
        fingerprint = (provider_ready, fallback_active, circuit_state)
        if _LAST_PROVIDER_STREAM_STATE.get(key) == fingerprint:
            continue
        _LAST_PROVIDER_STREAM_STATE[key] = fingerprint
        _STREAM_BUS.publish(
            event_type="provider_health",
            payload={
                "provider_name": provider_name,
                "provider_ready": provider_ready,
                "fallback_active": fallback_active,
                "circuit_state": circuit_state or "unknown",
                "latency_ms": payload_any.get("latency_ms"),
                "availability": payload_any.get("availability"),
                "timestamp": datetime.now(timezone.utc).isoformat(),
            },
        )
        if fallback_active:
            _STREAM_BUS.publish(
                event_type="degradation",
                payload={
                    "severity": "warning",
                    "domain": "provider",
                    "message": f"Fallback activo en {provider_name}.",
                },
            )
        if circuit_state and circuit_state.lower() == "open":
            _STREAM_BUS.publish(
                event_type="degradation",
                payload={
                    "severity": "critical",
                    "domain": "provider",
                    "message": f"Circuit breaker abierto en {provider_name}.",
                },
            )
