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

__all__ = [
    "SCANNER_IS_LEGACY",
    "LEAN_SIMULATOR_IS_INTERNAL_GBM",
]
