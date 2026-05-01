"""Atlas Code Quant — Modelos internos de oportunidad Radar (F6, shadow only).

Esta es la representación **interna** de Code Quant para una oportunidad
multi-símbolo emitida por el Radar (atlas_adapter). Es deliberadamente
**aislada** del modelo público (``atlas_adapter.routes.radar_schemas``):

* Code Quant nunca depende del módulo de rutas del Radar para su lógica
  de estrategia / risk; se queda con un *snapshot* propio.
* El mapeo desde el contrato público se hace en :func:`from_radar_payload`
  y :func:`batch_from_radar_payload` con normalización de tipos defensiva
  (acepta ``dict`` o instancias Pydantic).
* No se pierde información crítica: ``score``, ``classification``,
  ``direction``, ``horizon_min``, ``snapshot``, ``trace_id``,
  ``degradations_active``, ``source``.

Reglas duras (F6 — shadow only):
    * NO se invoca desde ``atlas_code_quant/api/main.py``.
    * NO se conecta a operations / execution / autonomy / risk en F6.
    * NO ejecuta trading. NO toca scanner. NO toca Tradier.
    * El flag ``ATLAS_RADAR_INTAKE_ENABLED`` (default ``False``) es
      doc-only en F6.

Ver también:
    * docs/ATLAS_CODE_QUANT_F6_RADAR_INTAKE_SHADOW.md
    * atlas_code_quant/intake/radar_client.py
    * atlas_adapter/routes/radar_schemas.py (contrato público)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Mapping

logger = logging.getLogger("atlas.code_quant.intake.radar.opportunity")


__all__ = [
    "RadarOpportunityInternal",
    "RadarOpportunityBatchInternal",
    "RadarIntakeDegradation",
    "from_radar_payload",
    "batch_from_radar_payload",
    "VALID_CLASSIFICATIONS",
    "VALID_DIRECTIONS",
    "VALID_SOURCES",
]


VALID_CLASSIFICATIONS: frozenset[str] = frozenset(
    {"high_conviction", "watchlist", "reject"}
)
VALID_DIRECTIONS: frozenset[str] = frozenset({"long", "short", "neutral"})
VALID_SOURCES: frozenset[str] = frozenset({"quant", "stub"})


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _to_dict(obj: Any) -> dict[str, Any]:
    """Convierte ``obj`` (Pydantic v2 / v1 / dict / mapping) a ``dict``.

    Tolerante: si no es nada de lo anterior, devuelve ``{}``.
    """
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return dict(obj)
    # Pydantic v2.
    dump = getattr(obj, "model_dump", None)
    if callable(dump):
        try:
            data = dump()
        except Exception:  # noqa: BLE001
            data = None
        if isinstance(data, dict):
            return data
    # Pydantic v1 / namedtuple-ish.
    dump_v1 = getattr(obj, "dict", None)
    if callable(dump_v1):
        try:
            data = dump_v1()
        except Exception:  # noqa: BLE001
            data = None
        if isinstance(data, dict):
            return data
    if isinstance(obj, Mapping):
        return dict(obj)
    return {}


def _safe_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    if f != f:  # NaN
        return default
    return f


def _safe_int_or_none(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _normalize_classification(value: Any) -> str:
    s = _safe_str(value).strip().lower()
    if s in VALID_CLASSIFICATIONS:
        return s
    return "reject"


def _normalize_direction(value: Any) -> str:
    s = _safe_str(value).strip().lower()
    if s in VALID_DIRECTIONS:
        return s
    return "neutral"


def _normalize_source(value: Any) -> str:
    s = _safe_str(value).strip().lower()
    if s in VALID_SOURCES:
        return s
    # Cualquier valor desconocido se trata como ``stub`` (degradación honesta).
    return "stub"


@dataclass(frozen=True)
class RadarIntakeDegradation:
    """Entrada de degradación normalizada (subset del contrato público).

    Siempre presenta los 4 campos canónicos. Si el payload original es
    inválido, se rellena con valores honestos.
    """

    code: str
    label: str
    severity: str  # "info" | "warning" | "critical"
    source: str

    @classmethod
    def from_payload(cls, payload: Any) -> "RadarIntakeDegradation":
        d = _to_dict(payload)
        sev = _safe_str(d.get("severity"), "warning").strip().lower()
        if sev not in ("info", "warning", "critical"):
            sev = "warning"
        return cls(
            code=_safe_str(d.get("code"), "UNKNOWN"),
            label=_safe_str(d.get("label"), ""),
            severity=sev,
            source=_safe_str(d.get("source"), ""),
        )

    def to_dict(self) -> dict[str, str]:
        return {
            "code": self.code,
            "label": self.label,
            "severity": self.severity,
            "source": self.source,
        }


@dataclass(frozen=True)
class RadarOpportunityInternal:
    """Snapshot inmutable de una oportunidad Radar consumida por Code Quant.

    Exposición intencionalmente simple para que strategies/risk puedan:
        * filtrar por ``score`` (``meets_min_score``),
        * elegir estrategia por ``asset_class`` (``is_etf``, ``is_equity``,
          ``is_index``),
        * cruzar end-to-end con el ``trace_id`` del Radar.

    NO contiene precios ni órdenes — esto NO es un signal de ejecución.
    """

    symbol: str
    asset_class: str
    sector: str | None
    optionable: bool
    score: float
    classification: str
    direction: str
    timestamp: str
    horizon_min: int | None
    snapshot: dict[str, Any]
    degradations_active: tuple[RadarIntakeDegradation, ...]
    source: str
    trace_id: str
    raw: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    # ------------------------------------------------------------------
    # Constructores
    # ------------------------------------------------------------------

    @classmethod
    def from_payload(cls, payload: Any) -> "RadarOpportunityInternal":
        """Construye desde un dict / Pydantic ``RadarOpportunity``.

        Defensivo: campos faltantes o malformados se sustituyen por
        valores neutros y no levantan excepción.
        """
        d = _to_dict(payload)
        snap = _to_dict(d.get("snapshot"))
        deg_list_raw = d.get("degradations_active") or []
        if not isinstance(deg_list_raw, (list, tuple)):
            deg_list_raw = []
        degs = tuple(RadarIntakeDegradation.from_payload(e) for e in deg_list_raw)
        return cls(
            symbol=_safe_str(d.get("symbol")).strip().upper(),
            asset_class=_safe_str(d.get("asset_class"), "unknown").strip().lower(),
            sector=(_safe_str(d.get("sector")).strip().lower() or None)
            if d.get("sector") is not None
            else None,
            optionable=bool(d.get("optionable")) if d.get("optionable") is not None else False,
            score=_safe_float(d.get("score")),
            classification=_normalize_classification(d.get("classification")),
            direction=_normalize_direction(d.get("direction")),
            timestamp=_safe_str(d.get("timestamp"), _utc_iso()),
            horizon_min=_safe_int_or_none(d.get("horizon_min")),
            snapshot=snap,
            degradations_active=degs,
            source=_normalize_source(d.get("source")),
            trace_id=_safe_str(d.get("trace_id")),
            raw=d,
        )

    # ------------------------------------------------------------------
    # Helpers de filtrado / clasificación (consumibles por strategies)
    # ------------------------------------------------------------------

    def meets_min_score(self, min_score: float) -> bool:
        return self.score >= float(min_score)

    @property
    def is_high_conviction(self) -> bool:
        return self.classification == "high_conviction"

    @property
    def is_watchlist(self) -> bool:
        return self.classification == "watchlist"

    @property
    def is_etf(self) -> bool:
        return self.asset_class == "etf"

    @property
    def is_equity(self) -> bool:
        return self.asset_class == "equity"

    @property
    def is_index(self) -> bool:
        return self.asset_class == "index"

    @property
    def is_live(self) -> bool:
        return self.source == "quant"

    @property
    def is_stub(self) -> bool:
        return self.source == "stub"

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "asset_class": self.asset_class,
            "sector": self.sector,
            "optionable": self.optionable,
            "score": self.score,
            "classification": self.classification,
            "direction": self.direction,
            "timestamp": self.timestamp,
            "horizon_min": self.horizon_min,
            "snapshot": dict(self.snapshot),
            "degradations_active": [d.to_dict() for d in self.degradations_active],
            "source": self.source,
            "trace_id": self.trace_id,
        }


@dataclass(frozen=True)
class RadarOpportunityBatchInternal:
    """Resultado interno de un fetch de oportunidades Radar.

    Refleja el envelope de :class:`RadarOpportunitiesResponse` pero con
    tipos internos. Incluye un campo ``intake_degradations`` propio que
    Code Quant añade cuando el intake mismo falla (timeouts, 5xx,
    payload inválido). Estas son **distintas** de las degradaciones
    globales del batch del Radar (``degradations_active``).
    """

    ok: bool
    source: str  # "quant" | "stub"
    timestamp: str
    universe_size: int
    evaluated: int
    succeeded: int
    failed: int
    returned: int
    limit: int
    min_score: float
    filters: dict[str, Any]
    items: tuple[RadarOpportunityInternal, ...]
    degradations_active: tuple[RadarIntakeDegradation, ...]
    intake_degradations: tuple[RadarIntakeDegradation, ...]
    trace_id: str

    # ------------------------------------------------------------------
    # Constructores
    # ------------------------------------------------------------------

    @classmethod
    def from_payload(
        cls,
        payload: Any,
        *,
        intake_degradations: Iterable[RadarIntakeDegradation] | None = None,
    ) -> "RadarOpportunityBatchInternal":
        d = _to_dict(payload)
        items_raw = d.get("items") or []
        if not isinstance(items_raw, (list, tuple)):
            items_raw = []
        items: list[RadarOpportunityInternal] = []
        for it in items_raw:
            try:
                items.append(RadarOpportunityInternal.from_payload(it))
            except Exception:  # noqa: BLE001 — un item malo no rompe el batch
                logger.exception("intake.radar: ítem inválido descartado")
        deg_raw = d.get("degradations_active") or []
        if not isinstance(deg_raw, (list, tuple)):
            deg_raw = []
        degs = tuple(RadarIntakeDegradation.from_payload(e) for e in deg_raw)
        intake_degs = tuple(intake_degradations or ())
        return cls(
            ok=bool(d.get("ok", True)),
            source=_normalize_source(d.get("source")),
            timestamp=_safe_str(d.get("timestamp"), _utc_iso()),
            universe_size=int(_safe_float(d.get("universe_size"))),
            evaluated=int(_safe_float(d.get("evaluated"))),
            succeeded=int(_safe_float(d.get("succeeded"))),
            failed=int(_safe_float(d.get("failed"))),
            returned=len(items),
            limit=int(_safe_float(d.get("limit"))),
            min_score=_safe_float(d.get("min_score")),
            filters=_to_dict(d.get("filters")),
            items=tuple(items),
            degradations_active=degs,
            intake_degradations=intake_degs,
            trace_id=_safe_str(d.get("trace_id")),
        )

    @classmethod
    def empty_with_degradation(
        cls,
        *,
        code: str,
        label: str,
        severity: str = "warning",
        source: str = "intake",
        trace_id: str = "",
    ) -> "RadarOpportunityBatchInternal":
        """Batch vacío con una degradación interna (cliente caído, timeout, etc.)."""
        deg = RadarIntakeDegradation(
            code=code, label=label, severity=severity, source=source
        )
        return cls(
            ok=False,
            source="stub",
            timestamp=_utc_iso(),
            universe_size=0,
            evaluated=0,
            succeeded=0,
            failed=0,
            returned=0,
            limit=0,
            min_score=0.0,
            filters={},
            items=(),
            degradations_active=(),
            intake_degradations=(deg,),
            trace_id=trace_id,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @property
    def has_intake_failure(self) -> bool:
        return bool(self.intake_degradations)

    def filter_by_min_score(self, min_score: float) -> tuple[RadarOpportunityInternal, ...]:
        return tuple(it for it in self.items if it.meets_min_score(min_score))

    def filter_by_asset_class(self, asset_class: str) -> tuple[RadarOpportunityInternal, ...]:
        ac = (asset_class or "").strip().lower()
        return tuple(it for it in self.items if it.asset_class == ac)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "source": self.source,
            "timestamp": self.timestamp,
            "universe_size": self.universe_size,
            "evaluated": self.evaluated,
            "succeeded": self.succeeded,
            "failed": self.failed,
            "returned": self.returned,
            "limit": self.limit,
            "min_score": self.min_score,
            "filters": dict(self.filters),
            "items": [it.to_dict() for it in self.items],
            "degradations_active": [d.to_dict() for d in self.degradations_active],
            "intake_degradations": [d.to_dict() for d in self.intake_degradations],
            "trace_id": self.trace_id,
        }


# ---------------------------------------------------------------------------
# Funciones de conveniencia (mapper público del módulo)
# ---------------------------------------------------------------------------


def from_radar_payload(payload: Any) -> RadarOpportunityInternal:
    """Mapea un ``RadarOpportunity`` (dict / Pydantic) → modelo interno."""
    return RadarOpportunityInternal.from_payload(payload)


def batch_from_radar_payload(
    payload: Any,
    *,
    intake_degradations: Iterable[RadarIntakeDegradation] | None = None,
) -> RadarOpportunityBatchInternal:
    """Mapea un ``RadarOpportunitiesResponse`` (dict / Pydantic) → batch interno."""
    return RadarOpportunityBatchInternal.from_payload(
        payload, intake_degradations=intake_degradations
    )
