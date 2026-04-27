"""Atlas Code Quant — Legacy flags (F1 scaffold).

Marcadores **informativos** introducidos en F1 para anclar el contrato de
componentes congelados. **No alteran el comportamiento de runtime.**

Reglas:
    * NO consultar estas flags desde código productivo en F1.
    * Sólo se usan como referencia documental y para herramientas de auditoría.
    * El cambio de semántica real (deprecation headers, cutover, 410 Gone)
      ocurre en fases posteriores (F3, F10) — no aquí.

Ver:
    - docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md (F0)
    - docs/ATLAS_CODE_QUANT_F1_REORG.md (F1)
    - docs/ATLAS_CODE_QUANT_F3_SCANNER_DEPRECATION.md (F3)
    - atlas_code_quant/legacy/README_SCANNER_FREEZE.md
"""

from __future__ import annotations

# El scanner heredado (atlas_code_quant/scanner/) queda lógicamente congelado
# desde F1. Esta flag es puramente informativa: no altera handlers, ni rutas,
# ni el comportamiento de operation_center.
SCANNER_IS_LEGACY: bool = True

# El simulador GBM interno NO es LEAN real. Renombrado físicamente en F2 a
# `atlas_code_quant.backtest.internal_gbm_simulator`. El path antiguo
# `atlas_code_quant.backtest.lean_simulator` queda como wrapper deprecated.
LEAN_SIMULATOR_IS_INTERNAL_GBM: bool = True

# ---------------------------------------------------------------------------
# F3 — Deprecación visible de endpoints scanner legacy.
# ---------------------------------------------------------------------------
#
# Habilita la disponibilidad de los endpoints HTTP heredados del scanner
# (rutas `/scanner/*` y `/api/v2/quant/scanner/*` definidos en
# `atlas_code_quant/api/main.py`).
#
# Estado en F3:
#     * Default = ``True``: los endpoints siguen 100% operativos y devuelven
#       el mismo status code, body y schema que antes.
#     * F3 sólo añade headers HTTP de deprecación (`Deprecation`, `Sunset`,
#       `Link`) y un log estructurado WARNING por cada hit.
#     * **Esta flag NO se consulta todavía** en runtime (`api/main.py` no la
#       lee como condicional en F3). Se documenta aquí como contrato
#       público para las fases siguientes.
#
# Roadmap (NO implementado en F3):
#     * Una fase posterior podrá leer esta flag y, cuando sea ``False``,
#       responder ``410 Gone`` o redirigir al sucesor canónico
#       (`/api/radar/*`).
#     * El cutover real depende de migrar a los consumidores documentados
#       en `atlas_code_quant/legacy/README_SCANNER_FREEZE.md`.
#
# Reglas duras:
#     * NO usar esta flag para deshabilitar lógica en F3.
#     * NO cambiar su default sin coordinación explícita con la fase
#       de cutover.
#     * NO interpretar su presencia como permiso para borrar endpoints.
ATLAS_LEGACY_SCANNER_ENABLED: bool = True

__all__ = [
    "SCANNER_IS_LEGACY",
    "LEAN_SIMULATOR_IS_INTERNAL_GBM",
    "ATLAS_LEGACY_SCANNER_ENABLED",
]
