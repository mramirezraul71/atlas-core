"""Atlas Code Quant — Runtime hook (opt-in) para shadow scanner→Radar (F9).

F9 introduce el primer punto donde el repositorio **lee en runtime** la
flag :data:`atlas_code_quant.config.legacy_flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`.

Diseño (verbatim del plan F9):

    def maybe_run_scanner_radar_shadow(scanner_candidates):
        if not ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED:
            return None
        return run_scanner_radar_shadow(scanner_candidates, ...)

Principios rectores
-------------------

* **F9 sigue siendo shadow**. El hook NUNCA muta ``scanner_candidates``,
  NUNCA filtra la lista que las estrategias usan, y NUNCA bloquea o
  corta la pipeline real. Es estrictamente observacional.
* **Default off**. ``ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`` sigue
  ``False``: el hook devuelve ``None`` sin tocar el Radar.
* **No enforcement**. F9 declara ``ATLAS_RADAR_FILTER_ENFORCED = False``
  pero NO la consulta. El gate duro queda para F10+.
* **Vive aquí, no en F8**. El hook se define en este módulo separado a
  propósito, para preservar la invariante F8
  (``test_no_runtime_module_consults_runtime_flag`` lee
  ``scanner_radar_shadow.py`` y exige que la flag NO aparezca textual
  en él).

Reglas duras (F9):

* NO se importa desde ``scanner/``, ``strategies/``, ``execution/`` ni
  ``api/`` (verificado por test estático en F9).
* NO se llama automáticamente desde ningún loop. Su llamada queda
  documentada en
  ``docs/ATLAS_CODE_QUANT_F9_SCANNER_RADAR_SHADOW_HOOK_AND_GATE_PREP.md``
  como "ready to call" para una fase posterior.
* NUNCA lanza al caller: cualquier fallo se traduce a degradaciones en
  el reporte (red de seguridad heredada de F8).
"""

from __future__ import annotations

import logging
from typing import Any, Iterable, Optional, Sequence

from atlas_code_quant.config import legacy_flags
from atlas_code_quant.intake.radar_client import RadarClient
from atlas_code_quant.intake.scanner_radar_filter import DEFAULT_MIN_SCORE
from atlas_code_quant.monitoring.scanner_radar_shadow import (
    ScannerRadarShadowReport,
    run_scanner_radar_shadow,
)

logger = logging.getLogger("atlas.code_quant.shadow.scanner_radar.hook")


__all__ = [
    "maybe_run_scanner_radar_shadow",
    "RUNTIME_FLAG_NAME",
    "ENFORCEMENT_FLAG_NAME",
]


#: Nombre canónico de la flag runtime que F9 consulta.
RUNTIME_FLAG_NAME: str = "ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED"

#: Nombre canónico de la flag de enforcement, doc-only en F9.
ENFORCEMENT_FLAG_NAME: str = "ATLAS_RADAR_FILTER_ENFORCED"


def maybe_run_scanner_radar_shadow(
    scanner_candidates: Sequence[Any] | Iterable[Any] | None,
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,
    emit_logs: bool = True,
) -> Optional[ScannerRadarShadowReport]:
    """Hook opt-in que ejecuta el shadow scanner→Radar SI la flag
    runtime está activa.

    Comportamiento:

    * Si :data:`legacy_flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`
      es ``False`` (default), devuelve ``None`` sin tocar nada.
      Específicamente:

        - no construye ni consulta el Radar,
        - no emite logs del logger F8 ``atlas.code_quant.shadow.scanner_radar``,
        - no consume ``scanner_candidates`` (no las itera).

    * Si la flag es ``True``, delega a
      :func:`atlas_code_quant.monitoring.scanner_radar_shadow.
      run_scanner_radar_shadow` con los kwargs provistos. La función
      delegada NUNCA lanza: cualquier fallo aparece como
      ``degradations`` dentro del :class:`ScannerRadarShadowReport`.

    Esta función NUNCA muta ``scanner_candidates``: su único contrato
    es leer-y-observar.

    Parameters
    ----------
    scanner_candidates:
        Iterable de candidatos del scanner (dicts u objetos con
        ``.symbol``). Aceptamos ``None`` y lo tratamos como entrada
        vacía cuando la flag está activa (consistente con F8).
    radar_client:
        Override del :class:`RadarClient`. Si ``None``, F7 construye
        uno por defecto.
    min_score:
        Umbral de aprobación del Radar.
    emit_logs:
        Forwardea a :func:`run_scanner_radar_shadow`. ``True`` por
        defecto.

    Returns
    -------
    Optional[ScannerRadarShadowReport]
        ``None`` si la flag está desactivada (caso por defecto).
        :class:`ScannerRadarShadowReport` poblado en otro caso.

    Notes
    -----
    Lectura de la flag a través de :mod:`legacy_flags` (atributo del
    módulo) en cada llamada. Esto permite a los tests mutar el módulo
    con ``monkeypatch.setattr`` sin reimportar este hook.
    """
    enabled = bool(getattr(legacy_flags, RUNTIME_FLAG_NAME, False))
    if not enabled:
        return None

    return run_scanner_radar_shadow(
        scanner_candidates,
        radar_client=radar_client,
        min_score=min_score,
        emit_logs=emit_logs,
    )
