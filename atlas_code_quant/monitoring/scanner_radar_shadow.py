"""Atlas Code Quant — Consumer runtime-safe scanner→Radar shadow (F8).

F8 introduce el primer *consumer* que sabe ejecutar el filtro shadow F7
contra un set real de candidatos del scanner y producir un reporte
agregado adecuado para logging / telemetría / journal interno.

Principio rector (F8):

    F8 observa y registra.
    F8 NO decide.
    F8 NO bloquea.
    F8 NO cambia el output efectivo del scanner.

Reglas duras:

    * NO modifica ``atlas_code_quant/api/main.py``.
    * NO toca scanner ni strategies ni execution.
    * NO toca operations live loops, autonomy FSM, risk, vision, locks.
    * NO toca atlas_adapter.
    * NO genera órdenes ni effects operativos.
    * El único side effect permitido es **logging estructurado** en el
      logger ``atlas.code_quant.shadow.scanner_radar``.
    * Reutiliza :func:`atlas_code_quant.intake.scanner_radar_filter.
      filter_scanner_candidates_with_radar` (F7).

Filosofía "no matar el proceso": esta función nunca lanza al caller.
Cualquier fallo se traduce a ``degradations`` en el reporte.

Ver también:
    * docs/ATLAS_CODE_QUANT_F8_SCANNER_RADAR_RUNTIME_SHADOW.md
    * atlas_code_quant/intake/scanner_radar_filter.py (F7)
    * atlas_code_quant/intake/radar_client.py (F6)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Sequence

from atlas_code_quant.intake.opportunity import RadarIntakeDegradation
from atlas_code_quant.intake.radar_client import RadarClient
from atlas_code_quant.intake.scanner_radar_filter import (
    DEFAULT_MIN_SCORE,
    ScannerRadarFilterResult,
    filter_scanner_candidates_with_radar,
)

logger = logging.getLogger("atlas.code_quant.shadow.scanner_radar")


__all__ = [
    "ScannerRadarShadowReport",
    "run_scanner_radar_shadow",
    "MAX_LOGGED_SYMBOLS",
]


#: Cuántos símbolos como máximo se incluyen en los logs estructurados
#: para evitar spam cuando el scanner produce listas grandes.
MAX_LOGGED_SYMBOLS: int = 25


# ---------------------------------------------------------------------------
# Reporte
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ScannerRadarShadowReport:
    """Reporte agregado de una corrida shadow scanner→Radar.

    No es un signal de ejecución. Es un *snapshot* de telemetría:
    cuántos candidatos propuso el scanner, cuántos aprobaría el Radar,
    qué divergencia hay y qué degradaciones aparecieron.
    """

    scanner_candidate_count: int
    radar_approved_count: int
    radar_rejected_count: int
    approved_symbols: tuple[str, ...]
    rejected_symbols: tuple[str, ...]
    degradations: tuple[RadarIntakeDegradation, ...]
    metadata: dict[str, Any]
    filter_result: ScannerRadarFilterResult = field(repr=False, compare=False)

    @property
    def divergence_ratio(self) -> float:
        """Fracción de candidatos del scanner que el Radar **rechazaría**.

        ``0.0`` si scanner_candidate_count == 0. Útil para vigilar la
        tasa de discrepancia scanner-vs-Radar a lo largo del tiempo.
        """
        if self.scanner_candidate_count <= 0:
            return 0.0
        return float(self.radar_rejected_count) / float(self.scanner_candidate_count)

    @property
    def has_batch_degradation(self) -> bool:
        return bool(self.degradations)

    def to_dict(self) -> dict[str, Any]:
        return {
            "scanner_candidate_count": self.scanner_candidate_count,
            "radar_approved_count": self.radar_approved_count,
            "radar_rejected_count": self.radar_rejected_count,
            "approved_symbols": list(self.approved_symbols),
            "rejected_symbols": list(self.rejected_symbols),
            "degradations": [d.to_dict() for d in self.degradations],
            "divergence_ratio": self.divergence_ratio,
            "metadata": dict(self.metadata),
        }


# ---------------------------------------------------------------------------
# Helpers internos
# ---------------------------------------------------------------------------


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _truncate(symbols: Sequence[str], limit: int = MAX_LOGGED_SYMBOLS) -> list[str]:
    if len(symbols) <= limit:
        return list(symbols)
    return list(symbols[:limit]) + [f"…(+{len(symbols) - limit})"]


def _empty_filter_result(min_score: float) -> ScannerRadarFilterResult:
    return ScannerRadarFilterResult(
        approved_by_radar=(),
        rejected_by_radar=(),
        radar_degradations=(),
        metadata={
            "trace_id": "",
            "timestamp": _utc_iso(),
            "min_score": float(min_score),
            "scanner_input": 0,
            "radar_returned": 0,
            "radar_universe_size": 0,
            "radar_source": "n/a",
            "approved_count": 0,
            "rejected_count": 0,
            "batch_failed": False,
        },
    )


def _build_unreachable_filter_result(
    candidates_count: int, min_score: float, exc: BaseException
) -> ScannerRadarFilterResult:
    """Resultado degradado defensivo cuando el filtro F7 fallara
    inesperadamente (no debería pasar — F7 ya tiene su red de
    seguridad). Lo mantenemos por *defense in depth*."""
    return ScannerRadarFilterResult(
        approved_by_radar=(),
        rejected_by_radar=(),
        radar_degradations=(
            RadarIntakeDegradation(
                code="RADAR_UNREACHABLE",
                label=f"Shadow consumer fallo inesperado: {type(exc).__name__}",
                severity="warning",
                source="atlas.code_quant.shadow.scanner_radar",
            ),
        ),
        metadata={
            "trace_id": "",
            "timestamp": _utc_iso(),
            "min_score": float(min_score),
            "scanner_input": int(candidates_count),
            "radar_returned": 0,
            "radar_universe_size": 0,
            "radar_source": "n/a",
            "approved_count": 0,
            "rejected_count": 0,
            "batch_failed": True,
        },
    )


def _emit_started(
    scanner_count: int, min_score: float, emit_logs: bool
) -> None:
    if not emit_logs:
        return
    logger.info(
        "scanner_radar_shadow_started",
        extra={
            "event": "scanner_radar_shadow_started",
            "scanner_count": scanner_count,
            "min_score": float(min_score),
        },
    )


def _emit_completed(
    report: ScannerRadarShadowReport, emit_logs: bool
) -> None:
    if not emit_logs:
        return
    logger.info(
        "scanner_radar_shadow_completed",
        extra={
            "event": "scanner_radar_shadow_completed",
            "scanner_count": report.scanner_candidate_count,
            "approved_count": report.radar_approved_count,
            "rejected_count": report.radar_rejected_count,
            "approved_symbols": _truncate(report.approved_symbols),
            "rejected_symbols": _truncate(report.rejected_symbols),
            "divergence_ratio": report.divergence_ratio,
            "min_score": report.metadata.get("min_score"),
            "trace_id": report.metadata.get("trace_id"),
        },
    )


def _emit_degraded(
    report: ScannerRadarShadowReport, emit_logs: bool
) -> None:
    if not emit_logs:
        return
    logger.warning(
        "scanner_radar_shadow_degraded",
        extra={
            "event": "scanner_radar_shadow_degraded",
            "scanner_count": report.scanner_candidate_count,
            "degradations": [d.to_dict() for d in report.degradations],
            "trace_id": report.metadata.get("trace_id"),
        },
    )


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def run_scanner_radar_shadow(
    scanner_candidates: Sequence[Any] | Iterable[Any],
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,
    emit_logs: bool = True,
) -> ScannerRadarShadowReport:
    """Ejecuta el filtro shadow F7 sobre ``scanner_candidates`` y agrega
    el resultado para telemetría.

    NUNCA lanza excepción hacia el caller. Cualquier fallo se traduce a
    degradaciones en el reporte. ``scanner_candidate_count`` siempre
    refleja la entrada original normalizada — incluso si Radar está
    caído — para que la tasa de cobertura sea visible.

    Parameters
    ----------
    scanner_candidates:
        Iterable de candidatos del scanner (dicts u objetos con
        ``.symbol``). Se acepta entrada vacía / ``None`` y se devuelve
        un reporte vacío.
    radar_client:
        Override del :class:`RadarClient`. Si ``None`` se construye uno
        nuevo dentro del filtro (F7).
    min_score:
        Umbral mínimo para que Radar marque un candidato como
        approved. Default ``70.0``.
    emit_logs:
        Si ``True`` (default), emite los eventos estructurados
        ``scanner_radar_shadow_started`` / ``...completed`` /
        ``...degraded``. ``False`` desactiva logging (útil para tests
        que validan ``emit_logs=False`` semantics).

    Returns
    -------
    ScannerRadarShadowReport
        Siempre poblado. Sin side effects aparte del logging.
    """
    # Materializa la entrada para poder contar incluso si filter F7
    # decide internamente normalizar / deduplicar.
    if scanner_candidates is None:
        materialized: list[Any] = []
    else:
        try:
            materialized = list(scanner_candidates)
        except TypeError:
            materialized = []

    _emit_started(len(materialized), min_score, emit_logs)

    if not materialized:
        empty = _empty_filter_result(min_score)
        report = _report_from_filter_result(0, empty)
        _emit_completed(report, emit_logs)
        return report

    try:
        filter_result = filter_scanner_candidates_with_radar(
            materialized,
            radar_client=radar_client,
            min_score=min_score,
        )
    except Exception as exc:  # noqa: BLE001 — última red de seguridad
        logger.exception(
            "scanner_radar_shadow: fallo inesperado en filter F7"
        )
        filter_result = _build_unreachable_filter_result(
            len(materialized), min_score, exc
        )

    report = _report_from_filter_result(len(materialized), filter_result)

    if report.has_batch_degradation:
        _emit_degraded(report, emit_logs)
    _emit_completed(report, emit_logs)
    return report


def _report_from_filter_result(
    scanner_input_count: int, result: ScannerRadarFilterResult
) -> ScannerRadarShadowReport:
    approved_syms = tuple(c.symbol for c in result.approved_by_radar)
    rejected_syms = tuple(c.symbol for c in result.rejected_by_radar)
    return ScannerRadarShadowReport(
        scanner_candidate_count=int(scanner_input_count),
        radar_approved_count=len(approved_syms),
        radar_rejected_count=len(rejected_syms),
        approved_symbols=approved_syms,
        rejected_symbols=rejected_syms,
        degradations=tuple(result.radar_degradations),
        metadata=dict(result.metadata),
        filter_result=result,
    )
