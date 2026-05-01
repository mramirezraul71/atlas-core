"""Atlas Code Quant — Filtro shadow scanner→Radar (F7).

F7 introduce el primer paso hacia el modelo objetivo:

    scanner = generador amplio  (recall)
    Radar   = filtro estricto    (precision)

Pero **sólo en sombra**. Esta función NO modifica el flujo efectivo de
estrategias: produce un :class:`ScannerRadarFilterResult` que la fase
posterior (F8+) podrá usar para enrutar candidatos. En F7 no se llama
desde ``api/main.py`` ni desde ningún loop de operations / execution.

Reglas duras (F7 — shadow only):

    * NO se invoca desde ``atlas_code_quant/api/main.py``.
    * NO se invoca desde ``atlas_code_quant/operations/*`` runtime loops.
    * NO se invoca desde ``atlas_code_quant/execution/*``.
    * NO se invoca desde ``atlas_code_quant/scanner/*``.
    * NO modifica órdenes ni ejecución.
    * Reutiliza :class:`atlas_code_quant.intake.radar_client.RadarClient`
      (introducido en F6); no abre cliente paralelo.
    * El flag ``ATLAS_RADAR_FILTER_SHADOW_ENABLED`` es doc-only en F7.

Filosofía "no matar el proceso": cualquier fallo se traduce a degradación
en el resultado; nunca se levanta excepción hacia fuera.

Ver también:
    * docs/ATLAS_CODE_QUANT_F7_SCANNER_RADAR_FILTER_SHADOW.md
    * atlas_code_quant/intake/opportunity.py
    * atlas_code_quant/intake/radar_client.py
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Mapping, Sequence

from atlas_code_quant.intake.opportunity import (
    RadarIntakeDegradation,
    RadarOpportunityBatchInternal,
    RadarOpportunityInternal,
)
from atlas_code_quant.intake.radar_client import (
    INTAKE_CODE_HTTP_ERROR,
    INTAKE_CODE_INVALID_PAYLOAD,
    INTAKE_CODE_TIMEOUT,
    INTAKE_CODE_UNREACHABLE,
    RadarClient,
)

logger = logging.getLogger("atlas.code_quant.intake.scanner_radar_filter")


__all__ = [
    "ScannerCandidateLike",
    "RadarFilteredCandidate",
    "ScannerRadarFilterResult",
    "filter_scanner_candidates_with_radar",
    "INTAKE_CODE_NO_DATA",
    "DEFAULT_MIN_SCORE",
]


# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------


INTAKE_CODE_NO_DATA: str = "RADAR_NO_DATA"
DEFAULT_MIN_SCORE: float = 70.0

#: Estos códigos provienen del fetch del Radar y son críticos al nivel de
#: batch (no por símbolo): si están presentes, el filtro NO rechaza por
#: política, sólo registra y degrada honestamente.
_BATCH_DEGRADATION_CODES: frozenset[str] = frozenset(
    {
        INTAKE_CODE_TIMEOUT,
        INTAKE_CODE_UNREACHABLE,
        INTAKE_CODE_HTTP_ERROR,
        INTAKE_CODE_INVALID_PAYLOAD,
    }
)


# ---------------------------------------------------------------------------
# Estructuras
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ScannerCandidateLike:
    """Vista mínima de un candidato del scanner para el filtro shadow.

    No es la estructura de runtime del scanner: es un proyectado mínimo
    estable que el filtro F7 sabe consumir. Soporta construcción desde
    ``dict`` (formato actual del ``OpportunityScannerService``) o desde
    cualquier objeto con atributo ``symbol``.

    El payload original se preserva en :attr:`raw` (no participa en
    ``__eq__`` para que dos candidatos del mismo símbolo se consideren
    iguales si su shape mínima coincide).
    """

    symbol: str
    asset_class: str = "unknown"
    score: float = 0.0
    raw: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    @classmethod
    def from_any(cls, obj: Any) -> "ScannerCandidateLike":
        """Construye desde dict / dataclass / objeto con .symbol.

        Tolera entradas malformadas: si no se puede extraer un símbolo
        usable, devuelve un candidato vacío (``symbol=""``) que el filtro
        descartará silenciosamente.
        """
        if isinstance(obj, ScannerCandidateLike):
            return obj
        if isinstance(obj, Mapping):
            d = dict(obj)
            sym = _safe_symbol(d.get("symbol"))
            ac = _safe_str(d.get("asset_class"), "unknown").lower()
            sc = _safe_float(d.get("score") or d.get("selection_score"))
            return cls(symbol=sym, asset_class=ac, score=sc, raw=d)
        # Objeto arbitrario con atributos.
        sym = _safe_symbol(getattr(obj, "symbol", ""))
        ac = _safe_str(getattr(obj, "asset_class", "unknown"), "unknown").lower()
        sc = _safe_float(getattr(obj, "score", 0.0))
        raw: dict[str, Any] = {}
        # Intento defensivo de capturar dict-like.
        try:
            raw_obj = getattr(obj, "__dict__", {}) or {}
            raw = dict(raw_obj)
        except Exception:  # noqa: BLE001 — defensivo
            raw = {}
        return cls(symbol=sym, asset_class=ac, score=sc, raw=raw)

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "asset_class": self.asset_class,
            "score": self.score,
        }


@dataclass(frozen=True)
class RadarFilteredCandidate:
    """Candidato del scanner enriquecido con la decisión del Radar.

    ``radar_opportunity`` es ``None`` cuando Radar no devolvió ninguna
    oportunidad para este símbolo (caso ``RADAR_NO_DATA``).
    """

    symbol: str
    scanner_candidate: ScannerCandidateLike
    radar_opportunity: RadarOpportunityInternal | None
    classification: str  # "high_conviction" | "watchlist" | "reject" | "no_data"
    score: float
    degradations: tuple[RadarIntakeDegradation, ...]

    @property
    def is_approved(self) -> bool:
        """``True`` si Radar lo clasifica como ``high_conviction`` o
        ``watchlist`` por encima del ``min_score`` aplicado."""
        return self.classification in ("high_conviction", "watchlist")

    @property
    def is_rejected(self) -> bool:
        return self.classification in ("reject", "no_data")

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "scanner_candidate": self.scanner_candidate.to_dict(),
            "radar_opportunity": (
                self.radar_opportunity.to_dict()
                if self.radar_opportunity is not None
                else None
            ),
            "classification": self.classification,
            "score": self.score,
            "degradations": [d.to_dict() for d in self.degradations],
        }


@dataclass(frozen=True)
class ScannerRadarFilterResult:
    """Resultado completo del filtro shadow.

    * ``approved_by_radar``: candidatos que la política del Radar acepta
      (high_conviction / watchlist con score ≥ ``min_score``).
    * ``rejected_by_radar``: el resto (reject explícito, no_data, score
      bajo).
    * ``radar_degradations``: degradaciones a nivel de batch / fetch
      (códigos `RADAR_TIMEOUT`, `RADAR_UNREACHABLE`, `RADAR_HTTP_ERROR`,
      `RADAR_INVALID_PAYLOAD`). Vacío en path feliz.
    * ``metadata``: dict con `trace_id`, `timestamp`, `min_score`,
      `radar_returned`, `radar_universe_size`, etc.
    """

    approved_by_radar: tuple[RadarFilteredCandidate, ...]
    rejected_by_radar: tuple[RadarFilteredCandidate, ...]
    radar_degradations: tuple[RadarIntakeDegradation, ...]
    metadata: dict[str, Any]

    @property
    def has_batch_degradation(self) -> bool:
        return bool(self.radar_degradations)

    def all_candidates(self) -> tuple[RadarFilteredCandidate, ...]:
        return self.approved_by_radar + self.rejected_by_radar

    def to_dict(self) -> dict[str, Any]:
        return {
            "approved_by_radar": [c.to_dict() for c in self.approved_by_radar],
            "rejected_by_radar": [c.to_dict() for c in self.rejected_by_radar],
            "radar_degradations": [d.to_dict() for d in self.radar_degradations],
            "metadata": dict(self.metadata),
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


def _safe_symbol(value: Any) -> str:
    s = _safe_str(value).strip().upper()
    return s


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    if f != f:  # NaN
        return default
    return f


def _normalize_candidates(
    candidates: Sequence[Any] | Iterable[Any] | None,
) -> list[ScannerCandidateLike]:
    """Normaliza la entrada y elimina duplicados de símbolo (preserva orden).

    Símbolos vacíos / inválidos se descartan silenciosamente.
    """
    if not candidates:
        return []
    out: list[ScannerCandidateLike] = []
    seen: set[str] = set()
    for c in candidates:
        try:
            cand = ScannerCandidateLike.from_any(c)
        except Exception:  # noqa: BLE001 — defensivo
            logger.exception(
                "scanner_radar_filter: candidato malformado descartado"
            )
            continue
        if not cand.symbol or cand.symbol in seen:
            continue
        seen.add(cand.symbol)
        out.append(cand)
    return out


def _index_radar_batch_by_symbol(
    batch: RadarOpportunityBatchInternal,
) -> dict[str, RadarOpportunityInternal]:
    return {opp.symbol: opp for opp in batch.items if opp.symbol}


def _classify(
    opp: RadarOpportunityInternal | None,
    *,
    min_score: float,
) -> tuple[str, float]:
    """Devuelve ``(classification, score)`` aplicando la política F7.

    Política:
        * Sin oportunidad Radar → ``("no_data", 0.0)``.
        * Con oportunidad pero ``score < min_score`` → ``("reject", score)``.
        * Si la clasificación nativa del Radar es ``reject`` → ``("reject",
          score)``.
        * Caso contrario → la clasificación nativa (``high_conviction`` /
          ``watchlist``) con su score.
    """
    if opp is None:
        return "no_data", 0.0
    if opp.score < float(min_score):
        return "reject", opp.score
    if opp.classification == "reject":
        return "reject", opp.score
    if opp.classification not in ("high_conviction", "watchlist"):
        # Cualquier otro valor inesperado: trata como reject (defensivo).
        return "reject", opp.score
    return opp.classification, opp.score


def _per_symbol_degradations(
    opp: RadarOpportunityInternal | None,
    classification: str,
) -> tuple[RadarIntakeDegradation, ...]:
    """Construye degradaciones por símbolo.

    * Si ``classification == "no_data"`` añade ``RADAR_NO_DATA`` info.
    * Hereda las ``degradations_active`` que el Radar trae para ese
      símbolo (ya normalizadas en F6).
    """
    if classification == "no_data":
        return (
            RadarIntakeDegradation(
                code=INTAKE_CODE_NO_DATA,
                label="Radar no tiene oportunidad para este símbolo",
                severity="info",
                source="atlas.code_quant.intake.scanner_radar_filter",
            ),
        )
    if opp is None:
        return ()
    return tuple(opp.degradations_active)


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def filter_scanner_candidates_with_radar(
    scanner_candidates: Sequence[Any] | Iterable[Any],
    radar_client: RadarClient | None = None,
    *,
    min_score: float = DEFAULT_MIN_SCORE,
) -> ScannerRadarFilterResult:
    """Filtra candidatos del scanner contra el Radar (shadow only en F7).

    Función **pura desde el punto de vista del runtime de producción**:
    no cambia ejecución, no toca ningún loop activo. Sólo consulta al
    Radar (vía :class:`RadarClient` de F6), clasifica cada candidato y
    devuelve un resultado estructurado.

    Filosofía "no matar el proceso":
        * cualquier fallo de fetch (red, timeout, 5xx, JSON inválido) se
          refleja en :attr:`ScannerRadarFilterResult.radar_degradations`,
        * en ese caso TODOS los candidatos se devuelven en
          ``rejected_by_radar`` con classification ``no_data``,
        * nunca se levanta excepción al caller.

    Parameters
    ----------
    scanner_candidates:
        Lista / iterable de candidatos del scanner (dicts o cualquier
        objeto con ``.symbol``). Símbolos vacíos / duplicados se
        ignoran silenciosamente.
    radar_client:
        Cliente Radar a usar. Si ``None``, se construye uno por defecto
        con env (``ATLAS_RADAR_BASE_URL`` / ``ATLAS_RADAR_TIMEOUT_MS``).
    min_score:
        Umbral mínimo de score para considerar approved. Default 70.0
        (refleja la política Radar high-conviction estándar). Se puede
        ajustar por estrategia en fases posteriores.

    Returns
    -------
    ScannerRadarFilterResult
        Siempre poblado; nunca lanza.
    """
    normalized = _normalize_candidates(scanner_candidates)
    timestamp = _utc_iso()

    if not normalized:
        return ScannerRadarFilterResult(
            approved_by_radar=(),
            rejected_by_radar=(),
            radar_degradations=(),
            metadata={
                "trace_id": "",
                "timestamp": timestamp,
                "min_score": float(min_score),
                "scanner_input": 0,
                "radar_returned": 0,
                "radar_universe_size": 0,
                "radar_source": "n/a",
            },
        )

    client = radar_client or RadarClient()
    symbols = [c.symbol for c in normalized]

    # Reutilizamos el snapshot multi-símbolo de F6: filtra por
    # ``symbols`` para que el batch del Radar venga acotado al universo
    # del scanner (el endpoint F5 acepta filtros).
    try:
        batch = client.fetch_opportunities({"symbols": symbols})
    except Exception:  # noqa: BLE001 — última red de seguridad
        logger.exception("scanner_radar_filter: fallo inesperado en fetch")
        batch = RadarOpportunityBatchInternal.empty_with_degradation(
            code=INTAKE_CODE_UNREACHABLE,
            label="Fallo inesperado al consultar Radar",
            severity="warning",
            source="atlas.code_quant.intake.scanner_radar_filter",
            trace_id="",
        )

    radar_degradations = tuple(batch.intake_degradations)
    radar_index = _index_radar_batch_by_symbol(batch)

    approved: list[RadarFilteredCandidate] = []
    rejected: list[RadarFilteredCandidate] = []

    # Si hubo degradación a nivel de batch, marcamos todo como ``no_data``
    # honestamente (no podemos pretender clasificar sin datos).
    has_batch_failure = bool(radar_degradations) or batch.has_intake_failure

    for cand in normalized:
        opp = None if has_batch_failure else radar_index.get(cand.symbol)
        if has_batch_failure:
            classification = "no_data"
            score = 0.0
        else:
            classification, score = _classify(opp, min_score=min_score)
        degs = _per_symbol_degradations(opp, classification)
        filtered = RadarFilteredCandidate(
            symbol=cand.symbol,
            scanner_candidate=cand,
            radar_opportunity=opp,
            classification=classification,
            score=score,
            degradations=degs,
        )
        if filtered.is_approved:
            approved.append(filtered)
        else:
            rejected.append(filtered)

    metadata: dict[str, Any] = {
        "trace_id": batch.trace_id or "",
        "timestamp": timestamp,
        "min_score": float(min_score),
        "scanner_input": len(normalized),
        "radar_returned": batch.returned,
        "radar_universe_size": batch.universe_size,
        "radar_source": batch.source,
        "approved_count": len(approved),
        "rejected_count": len(rejected),
        "batch_failed": has_batch_failure,
    }

    logger.info(
        "scanner_radar_filter shadow: in=%d approved=%d rejected=%d "
        "batch_failed=%s trace_id=%s",
        len(normalized),
        len(approved),
        len(rejected),
        has_batch_failure,
        metadata["trace_id"],
    )

    return ScannerRadarFilterResult(
        approved_by_radar=tuple(approved),
        rejected_by_radar=tuple(rejected),
        radar_degradations=radar_degradations,
        metadata=metadata,
    )
