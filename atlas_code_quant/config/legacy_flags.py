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
    - docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md (F4)
    - atlas_code_quant/legacy/README_SCANNER_FREEZE.md
    - atlas_code_quant/execution/README_TRADIER.md
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

# ---------------------------------------------------------------------------
# F4 — Canonicalización del stack Tradier.
# ---------------------------------------------------------------------------
#
# Atlas tiene actualmente DOS implementaciones Tradier coexistiendo:
#
#   1. CANÓNICA (runtime real):
#        ``atlas_code_quant.execution.tradier_execution``
#        (+ ``tradier_controls``, ``tradier_pdt_ledger``).
#      Único stack al que llegan rutas reales de órdenes (BrokerRouter,
#      SignalExecutor, /orders, OperationCenter, AutonExecutor,
#      LiveActivation, start_paper_trading) y que integra los locks
#      ``paper_only`` / ``full_live_globally_locked`` / dry-run defaults,
#      el ledger PDT y la reconciliación post-timeout.
#
#   2. LEGACY/PHASE1 (no es ruta de producción):
#        ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_executor``
#        ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_live``
#        ``atlas_options_brain_fase1.atlas_options_brain.providers.tradier_provider``
#      Snapshot de la fase de entrenamiento de Atlas Options Brain. Se
#      mantiene sólo para tests congelados de phase1 y como referencia
#      histórica de la API multileg de Tradier.
#
# Estas dos flags son **puramente documentales** en F4: ningún código de
# runtime las consulta. Sirven como contrato público para herramientas de
# auditoría y para fases posteriores que puedan unificar (con plan explícito
# de paridad y rollback).
#
# Reglas duras:
#     * NO importar el stack PHASE1 desde código de producción de
#       ``atlas_code_quant``.
#     * NO usar estas flags como condicionales de ruta en F4.
#     * NO cambiar sus valores sin coordinación explicita con la fase de
#       unificación.
ATLAS_TRADIER_CANONICAL_STACK: str = "atlas_code_quant"
ATLAS_TRADIER_PHASE1_LEGACY_STACK: str = "atlas_options_brain_fase1"

__all__ = [
    "SCANNER_IS_LEGACY",
    "LEAN_SIMULATOR_IS_INTERNAL_GBM",
    "ATLAS_LEGACY_SCANNER_ENABLED",
    "ATLAS_TRADIER_CANONICAL_STACK",
    "ATLAS_TRADIER_PHASE1_LEGACY_STACK",
]
