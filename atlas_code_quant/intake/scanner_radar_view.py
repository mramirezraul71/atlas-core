"""Atlas Code Quant — F10 adapter: scanner subordinado a Radar (API-level).

F10 hace efectiva la política de negocio:

    "scanner propone, Radar decide".

Este módulo NO ejecuta órdenes, NO toca execution / Tradier / live loops,
NO toca autonomy / risk / vision / locks. Sólo transforma el payload
crudo del scanner (``OpportunityScannerService.report(...)``) en un
payload **gated por Radar** apto para el endpoint ``/scanner/report``.

Modos de operación
------------------

El adapter soporta dos modos controlados por la flag
:data:`atlas_code_quant.config.legacy_flags.ATLAS_RADAR_FILTER_ENFORCED`:

* ``False`` (default): **modo compat**.

  - Conserva ``data.candidates`` igual que la implementación previa
    (lista del scanner, sin filtrar).
  - Añade información Radar en campos nuevos no destructivos:
    ``data.radar_filtered_candidates``, ``data.radar_rejected``,
    ``data.radar_degradations``, ``data.radar_meta``.
  - Rollback inmediato: la lista oficial sigue siendo el scanner.

* ``True``: **modo enforced**.

  - ``data.candidates`` pasa a ser **sólo** los candidatos aprobados
    por Radar (``approved_by_radar`` desde F7).
  - El payload incluye ``data.scanner_raw_candidates`` con la lista
    original del scanner (claramente etiquetada como "antes del gate
    Radar / legacy") para auditoría.
  - Rechazados, degradaciones y metadata Radar siguen disponibles.
  - Si Radar está caído / degradado: política conservadora
    documentada en F10 → ``data.candidates = []``,
    ``data.radar_degradations`` puebla la causa, y
    ``data.radar_meta.batch_failed = True``. Nunca se devuelve un
    candidato del scanner como si Radar lo hubiese aprobado.

Compatibilidad
--------------

* El shape básico de ``ScannerReportPayload`` (``generated_at``,
  ``status``, ``summary``, ``criteria``, ``universe``, ``candidates``,
  ``rejections``, ``activity``, ``current_work``, ``learning``,
  ``error``) se preserva.
* Pydantic ``ScannerReportPayload`` no declara ``extra="forbid"``,
  pero por simetría con la app actual y para no expandir el contrato
  HTTP por accidente, **los campos nuevos se devuelven como dict
  plano** desde el adapter; el handler los inyecta dentro del
  ``StdResponse.data`` final.

Reglas duras (F10)
------------------

* No se invoca a ``operations/*`` runtime loops.
* No se invoca a ``execution/*``.
* No se invoca a ``atlas_adapter/*``.
* No se invoca a ``autonomy/*``, ``risk/*``, ``vision/*``,
  ``production_guard/*``.
* La función nunca lanza al caller — cualquier fallo se traduce a
  ``radar_degradations`` y, si la flag está activa, a un payload
  conservador (sin candidatos).
* El hook F9 NO se usa aquí (su contrato sigue siendo "shadow only");
  F10 introduce su propio adapter dedicado al plano API.

Ver también
-----------

* docs/ATLAS_CODE_QUANT_F10_SCANNER_SUBORDINATED_TO_RADAR_ENFORCEMENT.md
* atlas_code_quant/intake/scanner_radar_filter.py (F7)
* atlas_code_quant/intake/radar_client.py (F6)
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any, Mapping, Sequence

from atlas_code_quant.config import legacy_flags
from atlas_code_quant.intake.radar_client import RadarClient
from atlas_code_quant.intake.scanner_radar_filter import (
    DEFAULT_MIN_SCORE,
    RadarFilteredCandidate,
    ScannerRadarFilterResult,
    filter_scanner_candidates_with_radar,
)

logger = logging.getLogger("atlas.code_quant.intake.scanner_radar_view")


__all__ = [
    "build_scanner_report_via_radar",
    "ENFORCEMENT_FLAG_NAME",
    "MODE_COMPAT",
    "MODE_ENFORCED",
]


#: Nombre canónico de la flag que controla enforcement (consultada en
#: runtime por F10).
ENFORCEMENT_FLAG_NAME: str = "ATLAS_RADAR_FILTER_ENFORCED"

MODE_COMPAT: str = "compat"
MODE_ENFORCED: str = "enforced"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _coerce_dict(value: Any) -> dict[str, Any]:
    """Convierte ``value`` a un dict mutable. ``None`` / no-mapping → ``{}``."""
    if value is None:
        return {}
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _coerce_list_of_dicts(value: Any) -> list[dict[str, Any]]:
    if not value:
        return []
    out: list[dict[str, Any]] = []
    for item in value:
        if isinstance(item, Mapping):
            out.append(dict(item))
        else:
            # No es dict — lo envolvemos en un dict mínimo para no
            # romper el contrato. ``raw`` preserva el item original
            # como string defensiva.
            out.append({"raw": repr(item)})
    return out


def _normalize_scanner_report(
    scanner_report: Any,
) -> dict[str, Any]:
    """Devuelve una **copia** del reporte del scanner con shape mínimo.

    Si la entrada es inválida, devuelve un esqueleto vacío que aún es
    compatible con :class:`ScannerReportPayload`.
    """
    base = _coerce_dict(scanner_report)

    return {
        "generated_at": str(base.get("generated_at") or _utc_iso()),
        "status": _coerce_dict(base.get("status")),
        "summary": _coerce_dict(base.get("summary")),
        "criteria": _coerce_list_of_dicts(base.get("criteria")),
        "universe": _coerce_dict(base.get("universe")),
        "candidates": _coerce_list_of_dicts(base.get("candidates")),
        "rejections": _coerce_list_of_dicts(base.get("rejections")),
        "activity": _coerce_list_of_dicts(base.get("activity")),
        "current_work": _coerce_dict(base.get("current_work")),
        "learning": _coerce_dict(base.get("learning")),
        "error": (
            None if base.get("error") is None else str(base.get("error"))
        ),
    }


def _approved_to_payload(
    approved: Sequence[RadarFilteredCandidate],
) -> list[dict[str, Any]]:
    """Convierte aprobados de Radar al shape esperado por
    ``ScannerReportPayload.candidates``.

    Estrategia:
        * Si el candidato del scanner trae ``raw`` (dict original del
          scanner), partimos de ese dict para preservar tuplas como
          ``selection_score``, ``ml_score``, ``method``, etc.
        * Sobreescribimos / añadimos ``radar_decision`` con la
          decisión del Radar para que el consumidor sepa que pasó el
          gate.
    """
    out: list[dict[str, Any]] = []
    for cand in approved:
        base = (
            dict(cand.scanner_candidate.raw)
            if cand.scanner_candidate.raw
            else {"symbol": cand.symbol}
        )
        base.setdefault("symbol", cand.symbol)
        base["radar_decision"] = {
            "approved": True,
            "classification": cand.classification,
            "score": cand.score,
            "trace_id": (
                cand.radar_opportunity.trace_id
                if cand.radar_opportunity is not None
                else ""
            ),
        }
        out.append(base)
    return out


def _rejected_to_payload(
    rejected: Sequence[RadarFilteredCandidate],
) -> list[dict[str, Any]]:
    """Lista de rechazados con razón. No se reusa shape del scanner —
    es información de auditoría. Siempre lleva ``approved=False``."""
    out: list[dict[str, Any]] = []
    for cand in rejected:
        out.append(
            {
                "symbol": cand.symbol,
                "approved": False,
                "classification": cand.classification,
                "score": cand.score,
                "degradations": [d.to_dict() for d in cand.degradations],
            }
        )
    return out


def _filter_result_to_payload(
    result: ScannerRadarFilterResult,
) -> dict[str, Any]:
    """Empaqueta el filtro F7 en un dict apto para JSON."""
    return {
        "approved_by_radar": _approved_to_payload(result.approved_by_radar),
        "rejected_by_radar": _rejected_to_payload(result.rejected_by_radar),
        "radar_degradations": [
            d.to_dict() for d in result.radar_degradations
        ],
        "metadata": dict(result.metadata),
    }


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def build_scanner_report_via_radar(
    scanner_report: Any,
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,
    enforced: bool | None = None,
) -> dict[str, Any]:
    """Construye el payload de ``/scanner/report`` con la decisión Radar
    aplicada.

    Parameters
    ----------
    scanner_report:
        Salida de ``OpportunityScannerService.report(activity_limit=...)``.
        Puede venir como ``dict`` o como objeto con ``.dict()``;
        cualquier otra cosa se trata como reporte vacío de forma
        defensiva.
    radar_client:
        Cliente Radar a usar. Si ``None``, F7 construye uno por defecto.
    min_score:
        Umbral mínimo para que Radar marque un candidato como
        approved. Default 70.0.
    enforced:
        Override explícito del modo. Si ``None`` (default), se consulta
        :data:`legacy_flags.ATLAS_RADAR_FILTER_ENFORCED` en runtime.

    Returns
    -------
    dict[str, Any]
        Payload listo para alimentar a ``ScannerReportPayload`` con los
        campos siguientes garantizados:

        * Campos heredados del scanner (``generated_at``, ``status``,
          ``summary``, ``criteria``, ``universe``, ``candidates``,
          ``rejections``, ``activity``, ``current_work``, ``learning``,
          ``error``).
        * Campos nuevos F10 (siempre presentes):

          - ``radar_filtered_candidates``: lista de aprobados por
            Radar con shape de ``data.candidates``. En modo enforced,
            es lo mismo que ``candidates``.
          - ``radar_rejected``: rechazados por Radar (shape de
            auditoría).
          - ``radar_degradations``: degradaciones del fetch Radar.
          - ``radar_meta``: dict con ``mode`` (``"compat"`` /
            ``"enforced"``), ``enforced`` (bool), ``min_score``,
            ``trace_id``, ``timestamp``, ``scanner_input``,
            ``approved_count``, ``rejected_count``, ``batch_failed``,
            ``radar_source``, ``policy``.
          - ``scanner_raw_candidates``: lista original del scanner
            (presente SOLO en modo enforced; en compat se omite porque
            ``data.candidates`` ya es la lista raw).

    Notes
    -----
    Política conservadora bajo degradación (modo enforced):
        Si Radar está caído / inválido / 5xx / timeout, NO se promueve
        ningún candidato del scanner. ``data.candidates = []``.
    """
    if enforced is None:
        enforced = bool(
            getattr(legacy_flags, ENFORCEMENT_FLAG_NAME, False)
        )

    payload = _normalize_scanner_report(scanner_report)
    raw_candidates = list(payload["candidates"])

    # Filtro F7 (siempre se ejecuta — la diferencia compat/enforced es
    # qué hacemos con su resultado de cara al campo "oficial"
    # ``candidates``).
    filter_result = filter_scanner_candidates_with_radar(
        raw_candidates,
        radar_client=radar_client,
        min_score=min_score,
    )
    radar_view = _filter_result_to_payload(filter_result)

    batch_failed = bool(
        filter_result.has_batch_degradation
        or filter_result.metadata.get("batch_failed")
    )

    if enforced:
        # En enforced, la verdad oficial es Radar.
        if batch_failed:
            # Política conservadora: ningún candidato pasa.
            payload["candidates"] = []
            policy = "conservative_block_on_degradation"
        else:
            payload["candidates"] = list(radar_view["approved_by_radar"])
            policy = "radar_gate_strict"
        # En enforced exponemos la lista raw del scanner para auditoría.
        payload["scanner_raw_candidates"] = raw_candidates
        mode = MODE_ENFORCED
    else:
        # En compat, la lista oficial sigue siendo el scanner.
        # No tocamos ``candidates``; añadimos vista Radar al lado.
        policy = "compat_observation_only"
        mode = MODE_COMPAT

    payload["radar_filtered_candidates"] = list(
        radar_view["approved_by_radar"]
    )
    payload["radar_rejected"] = list(radar_view["rejected_by_radar"])
    payload["radar_degradations"] = list(radar_view["radar_degradations"])

    radar_meta: dict[str, Any] = dict(radar_view["metadata"])
    radar_meta["mode"] = mode
    radar_meta["enforced"] = bool(enforced)
    radar_meta["batch_failed"] = batch_failed
    radar_meta["policy"] = policy
    radar_meta.setdefault(
        "scanner_input", radar_meta.get("scanner_input", len(raw_candidates))
    )
    radar_meta.setdefault(
        "approved_count",
        len(radar_view["approved_by_radar"]),
    )
    radar_meta.setdefault(
        "rejected_count",
        len(radar_view["rejected_by_radar"]),
    )
    payload["radar_meta"] = radar_meta

    logger.info(
        "scanner_radar_view: mode=%s enforced=%s scanner_input=%d "
        "approved=%d rejected=%d batch_failed=%s policy=%s",
        mode,
        enforced,
        radar_meta["scanner_input"],
        radar_meta["approved_count"],
        radar_meta["rejected_count"],
        batch_failed,
        policy,
    )

    return payload
