"""Motor batch F2: evalúa varios símbolos reutilizando el pipeline mono-símbolo del Radar.

Scoring: conservador y explicable (combinación de ``fast_pressure_score``,
``structural_confidence_score`` en [0,1] y un pequeño bono por clasificación
de snapshot). Documentado en informe F2.

No sustituye research; favorece coherencia y degradaciones honestas.
"""
from __future__ import annotations

import logging
import uuid
from dataclasses import dataclass
from typing import Any

from atlas_adapter.routes.radar_public import radar_build_dashboard_for_sse
from atlas_adapter.routes.radar_quant_mapper import pick_candidate
from atlas_adapter.services.universe_provider import UniverseEntry, UniverseProvider

from atlas_code_quant.config.feature_flags import AtlasFeatureFlags

logger = logging.getLogger("atlas.radar.batch")


def score_from_summary(body: dict[str, Any]) -> float:
    """Puntuación 0–100 alineada con meta del dashboard (no es alpha trading)."""
    meta = ((body.get("radar") or {}).get("signal") or {}).get("meta") or {}
    try:
        fast = float(meta.get("fast_pressure_score") or 0.0)
    except (TypeError, ValueError):
        fast = 0.0
    try:
        struct = float(meta.get("structural_confidence_score") or 0.0)
    except (TypeError, ValueError):
        struct = 0.0
    cls = str(meta.get("snapshot_classification") or "")
    # Pesos: estructura ligeramente menor que fast para no sobreponderar un solo canal.
    base = 100.0 * (0.55 * max(0.0, min(1.0, fast)) + 0.35 * max(0.0, min(1.0, struct)))
    bonus_map = {
        "fully_operable": 10.0,
        "operable_with_degradation": 4.0,
        "structural_only": 1.0,
        "demonstration_without_engine": 0.0,
        "non_operable": 0.0,
    }
    bonus = bonus_map.get(cls, 0.5)
    return round(min(100.0, base + bonus), 4)


def opportunity_classification(score: float, min_score: float, snap_cls: str) -> str:
    if score < min_score:
        return "below_threshold"
    if snap_cls == "fully_operable":
        return "high_conviction"
    if snap_cls == "operable_with_degradation":
        return "watchlist_degraded"
    if snap_cls == "structural_only":
        return "structural_watchlist"
    if snap_cls == "demonstration_without_engine":
        return "demonstration"
    if snap_cls == "non_operable":
        return "non_operable"
    return "watchlist"


def _horizon_minutes(cand: dict[str, Any] | None, *, stub: bool) -> int:
    if stub:
        return 60
    if not cand:
        return 45
    raw = cand.get("horizon_min")
    if raw is not None:
        try:
            return max(1, int(raw))
        except (TypeError, ValueError):
            pass
    tf = str(cand.get("timeframe") or "").strip().lower()
    if "0dte" in tf or tf in {"0dte", "0d"}:
        return 1
    if "week" in tf:
        return 10080
    if tf in {"1d", "day", "daily"}:
        return 1440
    return 60


def _slim_snapshot(body: dict[str, Any]) -> dict[str, Any]:
    sig = (body.get("radar") or {}).get("signal") or {}
    meta = sig.get("meta") or {}
    kpis = body.get("kpis") or {}
    return {
        "symbol": body.get("symbol"),
        "last_update": body.get("last_update"),
        "bias": sig.get("bias"),
        "snapshot_classification": meta.get("snapshot_classification"),
        "fast_pressure_score": meta.get("fast_pressure_score"),
        "structural_confidence_score": meta.get("structural_confidence_score"),
        "price": body.get("price") or kpis.get("price") or meta.get("price"),
    }


def _direction_for_quant(value: Any) -> str:
    direction = str(value or "").strip().lower()
    if direction in {"alcista", "bullish", "bull", "long", "buy", "up"}:
        return "alcista"
    if direction in {"bajista", "bearish", "bear", "short", "sell", "down"}:
        return "bajista"
    return "neutral"


def _copy_candidate_fields(cand: dict[str, Any] | None) -> dict[str, Any]:
    if not cand:
        return {}
    keys = (
        "price",
        "bid",
        "ask",
        "volume",
        "bar_volume",
        "predicted_move_pct",
        "confirmation",
        "market_regime",
        "regime",
        "iv_rank",
        "iv_rank_pct",
        "iv_hv_ratio",
        "liquidity_score",
        "skew_pct",
        "term_structure_slope",
        "options_thesis",
        "thesis",
        "order_flow",
        "strategy_key",
        "strategy_type",
        "has_options",
    )
    return {key: cand.get(key) for key in keys if cand.get(key) is not None}


def _merge_global_degradations(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_code: dict[str, dict[str, Any]] = {}
    for row in rows:
        for d in row.get("degradations_active") or []:
            if not isinstance(d, dict):
                continue
            code = str(d.get("code") or "")
            if code and code not in by_code:
                by_code[code] = d
    return list(by_code.values())


@dataclass(slots=True)
class RadarBatchResult:
    opportunities: list[dict[str, Any]]
    truncated: bool
    universe_evaluated: int
    trace_id: str
    global_degradations: list[dict[str, Any]]
    degraded_globally: bool
    any_quant: bool


async def build_radar_opportunities_batch(
    *,
    entries: list[UniverseEntry],
    min_score: float,
    limit: int,
    asset_class: str | None,
    sector: str | None,
    flags: AtlasFeatureFlags | None = None,
) -> RadarBatchResult:
    flags = flags or AtlasFeatureFlags()
    trace_id = str(uuid.uuid4())
    filt: list[UniverseEntry] = []
    ac = (asset_class or "").strip().lower() or None
    sec = (sector or "").strip().lower() or None
    for e in entries:
        if ac and e.asset_class.lower() != ac:
            continue
        if sec and e.sector.lower() != sec:
            continue
        filt.append(e)

    max_eval = max(1, min(len(filt), flags.radar_max_symbols_per_batch))
    to_eval = filt[:max_eval]
    truncated_universe = len(filt) > max_eval

    report_shared: dict[str, Any] | None = None
    raw_rows: list[dict[str, Any]] = []

    for ent in to_eval:
        body, live, rep = await radar_build_dashboard_for_sse(ent.symbol)
        if report_shared is None and rep is not None:
            report_shared = rep
        cand = pick_candidate(ent.symbol, report_shared) if report_shared else None
        meta = ((body.get("radar") or {}).get("signal") or {}).get("meta") or {}
        snap_cls = str(meta.get("snapshot_classification") or "")
        sc = score_from_summary(body)
        cls = opportunity_classification(sc, min_score, snap_cls)
        direction = _direction_for_quant(((body.get("radar") or {}).get("signal") or {}).get("bias") or "neutral")
        row = {
            "symbol": ent.symbol,
            "asset_class": ent.asset_class,
            "score": sc,
            "classification": cls,
            "timestamp": str(body.get("last_update") or ""),
            "horizon_min": _horizon_minutes(cand, stub=not live),
            "direction": direction,
            "snapshot": _slim_snapshot(body),
            "degradations_active": list(body.get("degradations_active") or []),
            "source": "quant" if live else "stub",
            "trace_id": f"{trace_id}:{ent.symbol}",
            **_copy_candidate_fields(cand),
        }
        raw_rows.append(row)
        logger.debug(
            "radar_batch: symbol=%s score=%s class=%s source=%s",
            ent.symbol,
            sc,
            cls,
            row["source"],
        )

    ranked = sorted(raw_rows, key=lambda r: float(r["score"]), reverse=True)
    filtered = [r for r in ranked if float(r["score"]) >= float(min_score)]
    lim = max(1, int(limit))
    out = filtered[:lim]
    truncated = truncated_universe or (len(filtered) > lim)
    any_quant = any(str(r.get("source")) == "quant" for r in raw_rows)

    gdeg = _merge_global_degradations(raw_rows)
    degraded = len(gdeg) > 0

    return RadarBatchResult(
        opportunities=out,
        truncated=truncated,
        universe_evaluated=len(to_eval),
        trace_id=trace_id,
        global_degradations=gdeg,
        degraded_globally=degraded,
        any_quant=any_quant,
    )


async def build_single_opportunity(
    *,
    symbol: str,
    min_score: float,
    flags: AtlasFeatureFlags | None = None,
) -> dict[str, Any] | None:
    flags = flags or AtlasFeatureFlags()
    sym = (symbol or "").strip().upper()
    if not sym:
        return None
    uni = UniverseProvider(flags=flags)
    ent = next((e for e in uni.refresh() if e.symbol == sym), None)
    if ent is None:
        return None
    batch = await build_radar_opportunities_batch(
        entries=[ent],
        min_score=min_score,
        limit=1,
        asset_class=None,
        sector=None,
        flags=flags,
    )
    if not batch.opportunities:
        return None
    return batch.opportunities[0]
