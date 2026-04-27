"""Test mínimo F2 — Compatibilidad de imports tras renombre `lean_simulator` → `internal_gbm_simulator`.

Este test NO ejecuta el motor matemático. Sólo verifica:

1. La ruta canónica nueva ``atlas_code_quant.backtest.internal_gbm_simulator``
   exporta los símbolos esperados.
2. La ruta histórica ``atlas_code_quant.backtest.lean_simulator`` sigue
   funcionando como wrapper deprecated.
3. Ambas rutas devuelven los **mismos objetos** (identidad), no copias.
4. La ruta histórica emite ``DeprecationWarning`` al importar.
5. El alias ``InternalGBMSimulator`` apunta a la misma clase que
   ``LeanSimulator`` (no se cambia el motor).

Ver: docs/ATLAS_CODE_QUANT_F2_INTERNAL_SIMULATOR_RENAME.md
"""

from __future__ import annotations

import importlib
import warnings


def test_canonical_path_exports() -> None:
    """La ruta canónica nueva expone toda la API pública sin warnings."""
    mod = importlib.import_module(
        "atlas_code_quant.backtest.internal_gbm_simulator"
    )
    assert hasattr(mod, "InternalGBMSimulator")
    assert hasattr(mod, "LeanSimulator")
    assert hasattr(mod, "SimConfig")
    assert hasattr(mod, "TradeRecord")
    # Alias canónico apunta exactamente al motor histórico.
    assert mod.InternalGBMSimulator is mod.LeanSimulator


def test_deprecated_shim_exports_same_objects() -> None:
    """El shim deprecated reexporta los MISMOS objetos que la ruta canónica."""
    canon = importlib.import_module(
        "atlas_code_quant.backtest.internal_gbm_simulator"
    )
    # Forzar reload del shim para capturar warnings limpios en cada test run.
    shim_name = "atlas_code_quant.backtest.lean_simulator"
    import sys

    sys.modules.pop(shim_name, None)
    with warnings.catch_warnings(record=True) as captured:
        warnings.simplefilter("always")
        shim = importlib.import_module(shim_name)
        deprecation_msgs = [
            str(w.message)
            for w in captured
            if issubclass(w.category, DeprecationWarning)
        ]
    assert any(
        "lean_simulator is deprecated" in m for m in deprecation_msgs
    ), (
        "El shim debe emitir DeprecationWarning. Mensajes capturados: "
        f"{deprecation_msgs}"
    )

    # Identidad estricta: los símbolos del shim son los mismos objetos.
    assert shim.LeanSimulator is canon.LeanSimulator
    assert shim.InternalGBMSimulator is canon.InternalGBMSimulator
    assert shim.SimConfig is canon.SimConfig
    assert shim.TradeRecord is canon.TradeRecord


def test_class_module_is_canonical() -> None:
    """``__module__`` de las clases públicas apunta al fichero canónico."""
    from atlas_code_quant.backtest.lean_simulator import (
        LeanSimulator,
        SimConfig,
        TradeRecord,
    )

    assert (
        LeanSimulator.__module__
        == "atlas_code_quant.backtest.internal_gbm_simulator"
    )
    assert (
        SimConfig.__module__
        == "atlas_code_quant.backtest.internal_gbm_simulator"
    )
    assert (
        TradeRecord.__module__
        == "atlas_code_quant.backtest.internal_gbm_simulator"
    )


def test_legacy_flag_advertises_internal_gbm() -> None:
    """La flag F1 documenta que el simulador NO es LEAN real."""
    from atlas_code_quant.config.legacy_flags import (
        LEAN_SIMULATOR_IS_INTERNAL_GBM,
    )

    assert LEAN_SIMULATOR_IS_INTERNAL_GBM is True
