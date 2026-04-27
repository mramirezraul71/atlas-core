"""atlas_code_quant.backtest.internal_gbm_simulator — Alias canónico (F1).

Este módulo es **sólo un alias de import** que prepara el terreno para F2,
donde ocurrirá el renombre físico de ``lean_simulator.py``.

En F1:
    * El fichero canónico real sigue siendo
      ``atlas_code_quant/backtest/lean_simulator.py`` (1224 LOC, GBM sintético).
    * Este módulo importa los símbolos públicos desde allí y los reexporta
      con un nombre honesto (``InternalGBMSimulator`` = ``LeanSimulator``).
    * Imports existentes ``from atlas_code_quant.backtest.lean_simulator import ...``
      siguen funcionando exactamente igual.

En F2 (planificado):
    * El contenido se moverá físicamente a este archivo.
    * ``lean_simulator.py`` quedará como wrapper con ``DeprecationWarning``.

No llamar "LEAN" a este simulador. NO es LEAN ni QuantConnect.
"""

from __future__ import annotations

# Reexport explícito desde la fuente actual (sin renombrar archivos en F1).
from atlas_code_quant.backtest.lean_simulator import (
    LeanSimulator as InternalGBMSimulator,
    SimConfig,
    TradeRecord,
)

__all__ = [
    "InternalGBMSimulator",
    "SimConfig",
    "TradeRecord",
]
