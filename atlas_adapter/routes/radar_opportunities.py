"""API F2: oportunidades multi-símbolo (detrás de ``ATLAS_RADAR_MULTI_SYMBOL_ENABLED``)."""
from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator
from typing import Any

from fastapi import APIRouter, HTTPException, Response
from fastapi.responses import StreamingResponse

from atlas_adapter.routes.radar_public import (
    _RADAR_SSE_HEARTBEAT_SEC,
    _RADAR_SSE_SNAPSHOT_SEC,
    _sse_event_block,
    _stream_headers,
    _utc_iso,
)
from atlas_adapter.routes.radar_quant_client import radar_quant_http_enabled
from atlas_adapter.routes.radar_schemas import (
    RadarOpportunitiesResponse,
    RadarOpportunity,
    RadarOpportunityDetailResponse,
)
from atlas_adapter.services.radar_batch_engine import (
    build_radar_opportunities_batch,
    build_single_opportunity,
)
from atlas_adapter.services.universe_provider import UniverseProvider

from atlas_code_quant.config.feature_flags import AtlasFeatureFlags

logger = logging.getLogger("atlas.radar.opportunities")

_MULTI_DISABLED_DETAIL: dict[str, Any] = {
    "ok": False,
    "code": "RADAR_MULTI_SYMBOL_DISABLED",
    "message": (
        "Radar multi-símbolo desactivado en este servidor. "
        "Establezca ATLAS_RADAR_MULTI_SYMBOL_ENABLED=1 para habilitar estos endpoints."
    ),
}


def _apply_no_store(response: Response) -> None:
    response.headers["Cache-Control"] = "no-store, max-age=0, must-revalidate"
    response.headers["Pragma"] = "no-cache"


def _require_multi(response: Response) -> AtlasFeatureFlags:
    flags = AtlasFeatureFlags()
    if not flags.radar_multi_symbol_enabled:
        raise HTTPException(
            status_code=503,
            detail=_MULTI_DISABLED_DETAIL,
            headers={
                "Cache-Control": "no-store, max-age=0, must-revalidate",
                "Pragma": "no-cache",
            },
        )
    _apply_no_store(response)
    return flags


def build_radar_opportunities_router() -> APIRouter:
    router = APIRouter(prefix="/api/radar", tags=["Radar Multi-Symbol"])

    @router.get(
        "/opportunities",
        response_model=RadarOpportunitiesResponse,
        responses={503: {"description": "Multi-símbolo desactivado (feature flag)."}},
    )
    async def list_opportunities(
        response: Response,
        limit: int = 50,
        min_score: float | None = None,
        asset_class: str | None = None,
        sector: str | None = None,
        source: str | None = None,
        mode: str | None = None,
    ) -> Any:
        """Ranking multi-símbolo. ``sector`` filtra cuando el catálogo lo permite; reservado si vacío."""
        flags = _require_multi(response)
        if source is not None and str(source).strip():
            logger.debug("list_opportunities: query source=%r (informativo F2)", source)
        if mode is not None and str(mode).strip():
            logger.debug("list_opportunities: query mode=%r (informativo F2)", mode)
        ms = float(min_score) if min_score is not None else float(flags.radar_min_score)
        lim = max(1, min(int(limit or 50), 500))
        uni = UniverseProvider(flags=flags)
        entries = uni.refresh()
        batch = await build_radar_opportunities_batch(
            entries=entries,
            min_score=ms,
            limit=lim,
            asset_class=asset_class,
            sector=sector,
            flags=flags,
        )
        opps = [RadarOpportunity.model_validate(o) for o in batch.opportunities]
        return RadarOpportunitiesResponse(
            ok=True,
            opportunities=opps,
            truncated=batch.truncated,
            universe_evaluated=batch.universe_evaluated,
            min_score=ms,
            trace_id=batch.trace_id,
            degraded_globally=batch.degraded_globally,
            global_degradations=batch.global_degradations,
            message=None,
        )

    @router.get(
        "/opportunities/{symbol}",
        response_model=RadarOpportunityDetailResponse,
        responses={
            503: {"description": "Multi-símbolo desactivado."},
            404: {"description": "Símbolo no en universo o bajo min_score."},
        },
    )
    async def get_opportunity(
        response: Response,
        symbol: str,
        min_score: float | None = None,
    ) -> Any:
        flags = _require_multi(response)
        sym = (symbol or "").strip().upper()
        ms = float(min_score) if min_score is not None else float(flags.radar_min_score)
        raw = await build_single_opportunity(symbol=sym, min_score=ms, flags=flags)
        if raw is None:
            raise HTTPException(
                status_code=404,
                detail={
                    "ok": False,
                    "code": "RADAR_OPPORTUNITY_NOT_FOUND",
                    "message": (
                        f"No hay oportunidad para {sym!r} (ausente del universo curado o score < min_score)."
                    ),
                },
            )
        return RadarOpportunityDetailResponse(
            ok=True,
            opportunity=RadarOpportunity.model_validate(raw),
            message=None,
        )

    @router.get("/stream/opportunities")
    async def stream_opportunities() -> StreamingResponse:
        flags = AtlasFeatureFlags()
        if not flags.radar_multi_symbol_enabled:
            raise HTTPException(
                status_code=503,
                detail=_MULTI_DISABLED_DETAIL,
                headers={
                    "Cache-Control": "no-store, max-age=0, must-revalidate",
                    "Pragma": "no-cache",
                },
            )
        live_cfg = radar_quant_http_enabled()

        async def _events() -> AsyncIterator[str]:
            yield ": atlas-radar-opportunities-sse\n\n"
            yield "retry: 3000\n\n"
            seq = 0
            last_snap = time.monotonic() - _RADAR_SSE_SNAPSHOT_SEC
            last_any_quant = False

            def _next_seq() -> int:
                nonlocal seq
                seq += 1
                return seq

            def _source_label(q: bool) -> str:
                return "quant" if q else "stub"

            uni = UniverseProvider(flags=flags)
            while True:
                now = time.monotonic()
                ts = _utc_iso()
                hb_env: dict[str, Any] = {
                    "type": "heartbeat",
                    "timestamp": ts,
                    "symbol": "*",
                    "source": _source_label(last_any_quant),
                    "sequence": _next_seq(),
                    "data": {
                        "heartbeat_interval_sec": _RADAR_SSE_HEARTBEAT_SEC,
                        "snapshot_interval_sec": _RADAR_SSE_SNAPSHOT_SEC,
                    },
                }
                yield _sse_event_block(hb_env["sequence"], "heartbeat", hb_env)

                if now - last_snap >= _RADAR_SSE_SNAPSHOT_SEC:
                    last_snap = now
                    try:
                        entries = uni.get_optionable_universe(
                            max_size=flags.radar_max_symbols_per_batch
                        )
                        batch = await build_radar_opportunities_batch(
                            entries=entries,
                            min_score=float(flags.radar_min_score),
                            limit=min(50, flags.radar_max_symbols_per_batch),
                            asset_class=None,
                            sector=None,
                            flags=flags,
                        )
                        last_any_quant = batch.any_quant
                        src = _source_label(batch.any_quant)
                        snap_env = {
                            "type": "universe_snapshot",
                            "timestamp": _utc_iso(),
                            "symbol": "*",
                            "source": src,
                            "sequence": _next_seq(),
                            "data": {
                                "symbols": [e.symbol for e in entries],
                                "count": len(entries),
                                "truncated": batch.truncated,
                                "trace_id": batch.trace_id,
                            },
                        }
                        yield _sse_event_block(snap_env["sequence"], "universe_snapshot", snap_env)
                        for opp in batch.opportunities[:40]:
                            ro = RadarOpportunity.model_validate(opp)
                            add_env = {
                                "type": "opportunity_added",
                                "timestamp": _utc_iso(),
                                "symbol": ro.symbol,
                                "source": ro.source,
                                "sequence": _next_seq(),
                                "data": {"opportunity": ro.model_dump()},
                            }
                            yield _sse_event_block(add_env["sequence"], "opportunity_added", add_env)
                    except Exception as exc:
                        logger.exception("stream/opportunities: error en ciclo: %s", exc)
                        err_env = {
                            "type": "universe_snapshot",
                            "timestamp": _utc_iso(),
                            "symbol": "*",
                            "source": "stub",
                            "sequence": _next_seq(),
                            "data": {"error": "batch_degraded", "detail": str(exc)[:200]},
                        }
                        yield _sse_event_block(err_env["sequence"], "universe_snapshot", err_env)

                await asyncio.sleep(_RADAR_SSE_HEARTBEAT_SEC)

        return StreamingResponse(
            _events(),
            media_type="text/event-stream",
            headers=_stream_headers(live=live_cfg),
        )

    return router
