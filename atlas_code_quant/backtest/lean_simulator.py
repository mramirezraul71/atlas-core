"""atlas_code_quant.backtest.lean_simulator — DEPRECATED compat shim (F2).

.. deprecated:: F2

    Este módulo es un wrapper de compatibilidad. El simulador interno
    real ha sido renombrado en F2 a
    ``atlas_code_quant.backtest.internal_gbm_simulator``.

    **Este módulo NO implementa LEAN/QuantConnect real.** El nombre
    histórico ``lean_simulator`` se mantiene SÓLO para no romper imports
    existentes (``atlas_code_quant.learning.pattern_lab``,
    ``scripts/generate_lean_dataset.py`` y código externo).

Migración recomendada::

    # Antes (sigue funcionando, pero deprecated):
    from atlas_code_quant.backtest.lean_simulator import (
        LeanSimulator, SimConfig, TradeRecord,
    )

    # Después (canónico desde F2):
    from atlas_code_quant.backtest.internal_gbm_simulator import (
        InternalGBMSimulator,  # alias del histórico LeanSimulator
        SimConfig,
        TradeRecord,
    )

Comportamiento:
    * Importar este módulo emite ``DeprecationWarning``.
    * Los símbolos públicos se reexportan **sin cambio de firma** desde
      ``internal_gbm_simulator``.
    * No hay refactor funcional. La matemática del motor GBM es idéntica.

Eliminación planeada: NO definida en F2. La eliminación física requerirá
una fase posterior con migración previa de los consumidores
(``learning/pattern_lab.py:497``, ``scripts/generate_lean_dataset.py:89``).
"""

from __future__ import annotations

import warnings as _warnings

# Aviso de deprecación al importar este módulo. ``stacklevel=2`` apunta al
# call-site del importador, no a esta línea.
_warnings.warn(
    "atlas_code_quant.backtest.lean_simulator is deprecated since F2; "
    "use atlas_code_quant.backtest.internal_gbm_simulator instead. "
    "Note: this module is NOT QuantConnect LEAN — it is the internal GBM "
    "synthetic simulator.",
    DeprecationWarning,
    stacklevel=2,
)

# Reexport explícito de la API pública canónica.
from atlas_code_quant.backtest.internal_gbm_simulator import (  # noqa: E402
    InternalGBMSimulator,
    LeanSimulator,
    SimConfig,
    TradeRecord,
)

__all__ = [
    # Histórico (la API que pattern_lab / scripts ya usan)
    "LeanSimulator",
    "SimConfig",
    "TradeRecord",
    # Canónico nuevo, expuesto también por compatibilidad descendente
    "InternalGBMSimulator",
]
