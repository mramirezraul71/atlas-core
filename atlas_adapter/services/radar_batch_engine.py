"""Atlas Radar — Motor de oportunidades multi-símbolo (F5).

Responsable de:

1. Recorrer un universo determinista (vía ``UniverseProvider``).
2. Construir, **por símbolo**, un summary del Radar reutilizando la pipeline ya
   probada (``radar_build_dashboard_for_sse``), capturando excepciones
   por-símbolo para no caer al primer fallo (degradación honesta).
3. Mapear ese summary a un :class:`RadarOpportunity` con score 0..100,
   clasificación por umbrales y direction derivada del bias canónico.
4. Aplicar filtros (``min_score``, ``asset_class``, ``sector``, ``optionable``)
   y orden estable (score desc, símbolo asc), con ``limit``.
5. Producir trazabilidad: ``trace_id`` por batch y por oportunidad,
   degradaciones globales agregadas, contadores de éxito/fallo.

Reglas duras (F5):

* NO ejecuta trading.
* NO toca ``operation_center``, ``production_guard``, locks ``paper_only`` /
  ``full_live_globally_locked``, ni endpoints existentes.
* NO depende de la flag ``ATLAS_RADAR_MULTI_SYMBOL_ENABLED`` (doc-only).
* Tolerante a fallo parcial: una excepción por símbolo NO tumba el batch.
* Registra todo en ``logger.exception`` con ``trace_id`` y símbolo.

Ver también:
    * docs/ATLAS_RADAR_F5_MULTI_SYMBOL_OPPORTUNITIES.md
    * atlas_adapter/services/universe_provider.py
    * atlas_adapter/routes/radar_public.py — ``radar_build_dashboard_for_sse``
"""

from __future__ import annotations

import logging
import uuid
from datetime import datetime, timezone
from typing import Any, Awaitable, Callable

from atlas_adapter.services.universe_provider import (
    UniverseEntry,
    UniverseProvider,
    default_universe_provider,
)

logger = logging.getLogger("atlas.radar.batch")

# Tipo del builder por símbolo. La firma respeta el contrato existente:
#   coro(symbol) -> (body: dict, live: bool, report: dict | None)
# Inyectable para tests sin tener que parchear el módulo de rutas.
SymbolBuilder = Callable[[str], Awaitable[tuple[dict[str, Any], bool, dict[str, Any] | None]]]


# ---------------------------------------------------------------------------
# Umbrales y mapeos canónicos
# ---------------------------------------------------------------------------

#: Score mínimo (0..100) para clasificar como ``high_conviction``.
HIGH_CONVICTION_MIN_SCORE: float = 70.0
#: Score mínimo (0..100) para clasificar como ``watchlist``.
WATCHLIST_MIN_SCORE: float = 40.0


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_float(v: Any, *, default: float = 0.0) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    if f != f:  # NaN
        return default
    return f


def _clamp_unit(v: float) -> float:
    if v < 0.0:
        return 0.0
    if v > 1.0:
        return 1.0
    return v


def _classification_for(score: float) -> str:
    if score >= HIGH_CONVICTION_MIN_SCORE:
        return "high_conviction"
    if score >= WATCHLIST_MIN_SCORE:
        return "watchlist"
    return "reject"


def _direction_from_bias(bias: Any) -> str:
    s = str(bias or "").strip().lower()
    if s in ("long", "largo", "alcista", "buy"):
        return "long"
    if s in ("short", "corto", "bajista", "sell"):
        return "short"
    return "neutral"


def _extract_signal_meta(summary: dict[str, Any]) -> dict[str, Any]:
    radar = summary.get("radar") if isinstance(summary, dict) else {}
    signal = radar.get("signal") if isinstance(radar, dict) else {}
    meta = signal.get("meta") if isinstance(signal, dict) else {}
    if not isinstance(meta, dict):
        return {}
    return meta


def _extract_signal_block(summary: dict[str, Any]) -> dict[str, Any]:
    radar = summary.get("radar") if isinstance(summary, dict) else {}
    signal = radar.get("signal") if isinstance(radar, dict) else {}
    if not isinstance(signal, dict):
        return {}
    return signal


def compute_opportunity_score(summary: dict[str, Any]) -> float:
    """Score compuesto 0..100 derivado del summary canónico.

    Composición conservadora: 60% fast_pressure, 40% structural_confidence.
    Ambos vienen normalizados 0..1 desde ``build_dashboard_summary``.
    """
    meta = _extract_signal_meta(summary)
    fp = _clamp_unit(_safe_float(meta.get("fast_pressure_score")))
    sc = _clamp_unit(_safe_float(meta.get("structural_confidence_score")))
    composite = (0.6 * fp) + (0.4 * sc)
    return round(composite * 100.0, 2)


def build_snapshot_payload(summary: dict[str, Any]) -> dict[str, Any]:
    """Subset canónico para ``RadarOpportunitySnapshot`` desde un summary."""
    meta = _extract_signal_meta(summary)
    signal = _extract_signal_block(summary)
    ts = signal.get("timestamp") or summary.get("last_update") or _utc_iso()
    return {
        "timestamp": str(ts),
        "snapshot_classification": str(meta.get("snapshot_classification") or "demonstration_without_engine"),
        "fast_pressure_score": round(_clamp_unit(_safe_float(meta.get("fast_pressure_score"))), 4),
        "structural_confidence_score": round(_clamp_unit(_safe_float(meta.get("structural_confidence_score"))), 4),
        "fast_structural_alignment": round(_clamp_unit(_safe_float(meta.get("fast_structural_alignment"))), 4),
        "fast_structural_divergence_score": round(
            _clamp_unit(_safe_float(meta.get("fast_structural_divergence_score"))), 4
        ),
        "horizon_conflict": bool(meta.get("horizon_conflict") or False),
        "cross_horizon_alignment": str(meta.get("cross_horizon_alignment") or "-"),
        "bias": str(signal.get("bias") or "neutral"),
    }


def opportunity_from_summary(
    *,
    entry: UniverseEntry,
    summary: dict[str, Any],
    live: bool,
    batch_trace_id: str,
) -> dict[str, Any]:
    """Construye un dict con la forma de ``RadarOpportunity`` (sin validar)."""
    score = compute_opportunity_score(summary)
    snap = build_snapshot_payload(summary)
    bias = snap.get("bias")
    degradations = summary.get("degradations_active") or []
    if not isinstance(degradations, list):
        degradations = []
    item = {
        "symbol": entry.symbol,
        "asset_class": entry.asset_class,
        "sector": entry.sector,
        "optionable": bool(entry.optionable),
        "score": score,
        "classification": _classification_for(score),
        "direction": _direction_from_bias(bias),
        "timestamp": snap["timestamp"],
        "horizon_min": None,
        "snapshot": snap,
        "degradations_active": list(degradations),
        "source": "quant" if live else "stub",
        "trace_id": f"{batch_trace_id}:{entry.symbol}",
    }
    return item


# ---------------------------------------------------------------------------
# Resultado del batch (estructurado, no-Pydantic para evitar dep circular)
# ---------------------------------------------------------------------------


class RadarBatchResult:
    """Resultado de una corrida del batch engine.

    Atributos públicos accesibles por las rutas para construir la respuesta
    HTTP y los eventos SSE multi-símbolo.
    """

    __slots__ = (
        "items",
        "evaluated",
        "succeeded",
        "failed",
        "universe_size",
        "any_live",
        "trace_id",
        "global_degradations",
        "filters",
        "limit",
        "min_score",
        "timestamp",
    )

    def __init__(
        self,
        *,
        items: list[dict[str, Any]],
        evaluated: int,
        succeeded: int,
        failed: int,
        universe_size: int,
        any_live: bool,
        trace_id: str,
        global_degradations: list[dict[str, Any]],
        filters: dict[str, Any],
        limit: int,
        min_score: float,
        timestamp: str,
    ) -> None:
        self.items = items
        self.evaluated = evaluated
        self.succeeded = succeeded
        self.failed = failed
        self.universe_size = universe_size
        self.any_live = any_live
        self.trace_id = trace_id
        self.global_degradations = global_degradations
        self.filters = filters
        self.limit = limit
        self.min_score = min_score
        self.timestamp = timestamp

    @property
    def returned(self) -> int:
        return len(self.items)

    @property
    def source(self) -> str:
        return "quant" if self.any_live else "stub"


# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------


class RadarBatchEngine:
    """Recorre el universo y genera oportunidades multi-símbolo.

    Es **stateless** entre llamadas. Inyectable: el ``symbol_builder`` puede
    sobreescribirse en tests para no depender del runtime de Quant.
    """

    def __init__(
        self,
        *,
        universe: UniverseProvider | None = None,
        symbol_builder: SymbolBuilder | None = None,
    ) -> None:
        self._universe = universe or default_universe_provider()
        self._symbol_builder = symbol_builder

    @property
    def universe(self) -> UniverseProvider:
        return self._universe

    async def _build_symbol(
        self, symbol: str
    ) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        """Resuelve el builder lazy para no exigir import de rutas en tests."""
        if self._symbol_builder is not None:
            return await self._symbol_builder(symbol)
        # Import diferido para evitar ciclo en tiempo de import.
        from atlas_adapter.routes.radar_public import (  # noqa: WPS433 (intencional)
            radar_build_dashboard_for_sse,
        )

        return await radar_build_dashboard_for_sse(symbol)

    async def compute_opportunities(
        self,
        *,
        limit: int = 50,
        min_score: float = 0.0,
        asset_class: str | None = None,
        sector: str | None = None,
        optionable: bool | None = None,
        symbols: list[str] | None = None,
    ) -> RadarBatchResult:
        """Genera oportunidades aplicando filtros sobre el universo.

        Tolerante a fallo: una excepción por símbolo se registra y no aborta
        el batch. Se incrementa ``failed`` y se añade una entrada de
        degradación global ``OPPORTUNITY_BUILD_FAILED`` (severity warning).
        """
        batch_trace_id = uuid.uuid4().hex[:12]
        ts = _utc_iso()
        # Limit ≤ 0 ⇒ devolver vacío (decisión consciente, evita full barrido).
        try:
            lim = max(0, int(limit))
        except (TypeError, ValueError):
            lim = 50
        try:
            min_s = max(0.0, float(min_score))
        except (TypeError, ValueError):
            min_s = 0.0

        # Universo filtrado (sin recortar todavía con limit; el limit final
        # aplica tras scoring para no excluir prematuramente buenos picks).
        candidate_entries = self._universe.list(
            asset_class=asset_class,
            sector=sector,
            optionable=optionable,
            enabled_only=True,
            symbols=symbols,
        )
        universe_size = len(candidate_entries)

        items: list[dict[str, Any]] = []
        succeeded = 0
        failed = 0
        any_live = False
        global_degradations: list[dict[str, Any]] = []

        for entry in candidate_entries:
            try:
                summary, live, _report = await self._build_symbol(entry.symbol)
                if not isinstance(summary, dict):
                    raise TypeError(
                        f"radar_build_dashboard_for_sse devolvió summary no dict para {entry.symbol!r}"
                    )
                opp = opportunity_from_summary(
                    entry=entry,
                    summary=summary,
                    live=bool(live),
                    batch_trace_id=batch_trace_id,
                )
                items.append(opp)
                succeeded += 1
                if live:
                    any_live = True
            except Exception as exc:  # noqa: BLE001 — degradación honesta por símbolo
                failed += 1
                logger.exception(
                    "radar_batch: fallo construyendo oportunidad symbol=%s trace_id=%s err=%r",
                    entry.symbol,
                    batch_trace_id,
                    exc,
                )
                global_degradations.append(
                    {
                        "code": "OPPORTUNITY_BUILD_FAILED",
                        "label": (
                            f"No se pudo construir oportunidad para {entry.symbol}: {type(exc).__name__}"
                        ),
                        "severity": "warning",
                        "source": "batch",
                    }
                )

        if not any_live and universe_size > 0:
            global_degradations.append(
                {
                    "code": "QUANT_UNREACHABLE_BATCH",
                    "label": (
                        "Ningún símbolo del batch obtuvo summary en vivo del motor Quant; "
                        "se devuelven snapshots stub honestos."
                    ),
                    "severity": "warning",
                    "source": "quant",
                }
            )

        # Filtro min_score, ordenamiento estable y limit final.
        filtered = [it for it in items if _safe_float(it.get("score")) >= min_s]
        filtered.sort(key=lambda it: (-_safe_float(it.get("score")), str(it.get("symbol") or "")))
        if lim > 0:
            filtered = filtered[:lim]

        filters_applied: dict[str, Any] = {
            "asset_class": asset_class,
            "sector": sector,
            "optionable": optionable,
            "symbols": list(symbols) if symbols else None,
        }

        return RadarBatchResult(
            items=filtered,
            evaluated=universe_size,
            succeeded=succeeded,
            failed=failed,
            universe_size=universe_size,
            any_live=any_live,
            trace_id=batch_trace_id,
            global_degradations=global_degradations,
            filters=filters_applied,
            limit=lim,
            min_score=min_s,
            timestamp=ts,
        )

    async def compute_opportunity_for(
        self, symbol: str
    ) -> dict[str, Any] | None:
        """Construye una sola oportunidad para ``symbol``.

        Retorna ``None`` si el símbolo no está en el universo (404 desde la
        ruta). Si está en el universo y el builder falla, propaga la
        excepción al caller (la ruta decide el envelope).
        """
        entry = self._universe.get(symbol)
        if entry is None:
            return None
        batch_trace_id = uuid.uuid4().hex[:12]
        summary, live, _report = await self._build_symbol(entry.symbol)
        return opportunity_from_summary(
            entry=entry,
            summary=summary,
            live=bool(live),
            batch_trace_id=batch_trace_id,
        )


__all__ = [
    "HIGH_CONVICTION_MIN_SCORE",
    "WATCHLIST_MIN_SCORE",
    "RadarBatchEngine",
    "RadarBatchResult",
    "SymbolBuilder",
    "build_snapshot_payload",
    "compute_opportunity_score",
    "opportunity_from_summary",
]
