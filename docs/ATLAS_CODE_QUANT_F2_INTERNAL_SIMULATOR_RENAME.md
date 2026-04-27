# Atlas Code Quant — F2 Rename: `lean_simulator` → `internal_gbm_simulator`

**Fase:** F2 (alcance reducido y vinculante: sólo normalización semántica del simulador interno).
**Repo:** `mramirezraul71/atlas-core`
**Rama:** `variante/nueva`
**HEAD previo:** `31ef2c04` (F1 scaffolding) → `458a46cd` (F0) → `41f86833` (origen).
**Commit F2 planificado:** `refactor: F2 rename internal GBM simulator and preserve lean compatibility shim`
**Push:** prohibido. Sólo commit local.

> **Cumplimiento de instrucciones F2:** sólo normalización del falso "LEAN" en el path del backtester interno. No se toca matemática, no se toca Radar, no se toca scanner, no se toca execution/Tradier, no se toca runtime, no se toca endpoints. F5+ trata Radar multi-símbolo / batch engine / `/api/radar/opportunities`.

---

## 1. Motivo del cambio

Antes de F2, el repositorio tenía un fichero llamado **`atlas_code_quant/backtest/lean_simulator.py`** y una clase **`LeanSimulator`** que **NO** son LEAN ni QuantConnect:

- El motor es un **simulador GBM sintético interno** (Geometric Brownian Motion + régimen Markov + indicadores `numpy`).
- Su propio docstring lo declara: *"sin TA-lib, sin yfinance, sin Tradier"*.
- No invoca runtime LEAN, no se conecta a QuantConnect ni a un motor externo.

El nombre histórico `lean_simulator` ha causado **confusión semántica**: documentos, scripts y consumidores lo tratan como si fuese una integración LEAN real. F2 corrige esto sin tocar comportamiento.

> Plan F0/F1 ya documentaba este renombre. F1 dejó un alias re-export como preparación. F2 ejecuta la consolidación real y añade el wrapper de compatibilidad descendente.

---

## 2. Diferencia explícita: simulador interno vs QuantConnect LEAN real

| Aspecto | `internal_gbm_simulator.py` (este módulo) | LEAN / QuantConnect real |
|---|---|---|
| Naturaleza | Simulador GBM sintético + régimen Markov | Motor de algoritmos backtest/live productivo |
| Datos de entrada | Generados internamente con `numpy` | Datos históricos / live reales |
| Dependencias | `numpy`, `pandas` | LEAN engine (C# / Docker), datos QuantConnect |
| Pipeline | Replica `LiveLoop` Atlas → `trades_df`, `features_df` | Algoritmo Python/C# con eventos OnData/OnOrderEvent |
| Propósito en Atlas | Generar datasets de entrenamiento sin servicios externos | (futuro) Backtest/live institucional con ejecución externa |
| Dónde reside en Atlas | `atlas_code_quant/backtest/internal_gbm_simulator.py` | `atlas_code_quant/lean/` (scaffold F1, decisión arquitectónica en F9) |

**Implicación operativa:** cualquier mención a "LEAN" en logs, métricas, dashboards o documentos públicos debe referirse exclusivamente al runtime real (cuando exista). El simulador interno se debe llamar "Internal GBM Simulator" en superficie pública.

---

## 3. Archivos tocados (alcance F2)

| Archivo | Acción | Notas |
|---|---|---|
| `atlas_code_quant/backtest/lean_simulator.py` | **Movido vía `git mv`** a `internal_gbm_simulator.py` | Historia preservada. Contenido movido sin cambios funcionales. |
| `atlas_code_quant/backtest/internal_gbm_simulator.py` | **Borrado el alias F1** (35 líneas) y reemplazado por el archivo movido (≈1245 LOC con docstring F2 + alias canónico al final). | Es ahora el **único fichero canónico** del simulador interno. |
| `atlas_code_quant/backtest/lean_simulator.py` (nuevo) | **Wrapper deprecated** (~70 líneas) que reexporta y emite `DeprecationWarning`. | Mantiene compat con consumidores existentes. |
| `atlas_code_quant/config/legacy_flags.py` | Comentario actualizado en la flag `LEAN_SIMULATOR_IS_INTERNAL_GBM` para reflejar el renombre realizado en F2. | No cambia el valor (sigue `True`). |
| `docs/ATLAS_CODE_QUANT_F1_REORG.md` | Referencia cruzada: el alias F1 fue superseded por F2. | Sólo §2.3 actualizado. |
| `docs/ATLAS_CODE_QUANT_F2_INTERNAL_SIMULATOR_RENAME.md` | **Nuevo** (este documento). | |
| `atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py` | **Nuevo** test de import/alias (4 tests, sin tocar matemática). | |

**Archivos NO tocados en F2** (regla dura):

- `atlas_code_quant/learning/pattern_lab.py` (línea 497 mantiene su `from atlas_code_quant.backtest.lean_simulator import LeanSimulator` lazy — sigue funcionando vía shim).
- `scripts/generate_lean_dataset.py` (línea 89 mantiene `from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig` — sigue funcionando vía shim).
- Toda la matemática del simulador (`_ema`, `_rsi`, `_macd`, `_atr`, `_bollinger_pct`, `_obv_norm`, `_volume_ratio`, `_adx_simple`, `_hurst`, `_cvd`, `_build_full_universe`, métodos de `LeanSimulator`).
- Tests existentes del scanner/options/risk/radar.
- Endpoints de `api/main.py`.
- Locks `paper_only`, `full_live_globally_locked`.
- Tradier execution / controls / pdt_ledger / broker_router / paper_broker.
- Radar institucional (`atlas_adapter/`).
- Operation_center, production_guard, journal.
- `atlas_code_quant/scanner/`.

---

## 4. Estrategia de compatibilidad

### 4.1 Superficie pública canónica (F2)

Definida en `atlas_code_quant/backtest/internal_gbm_simulator.py`:

```python
__all__ = [
    "InternalGBMSimulator",  # nuevo nombre canónico (alias del histórico)
    "LeanSimulator",         # nombre histórico, conservado por compat
    "SimConfig",
    "TradeRecord",
]

InternalGBMSimulator = LeanSimulator  # identidad estricta
```

- **`InternalGBMSimulator`** es el nombre canónico nuevo. Nuevo código debe usar este.
- **`LeanSimulator`** se conserva como nombre interno de la clase. **No se toca firma ni motor.** Los consumidores existentes siguen funcionando.
- `SimConfig` y `TradeRecord` se exponen sin renombre.
- `_build_full_universe` y los helpers (`_ema`, `_rsi`, etc.) **no son API pública** — siguen internos al módulo.

### 4.2 Wrapper deprecated (`lean_simulator.py` nuevo, ~70 LOC)

```python
import warnings as _warnings
_warnings.warn(
    "atlas_code_quant.backtest.lean_simulator is deprecated since F2; "
    "use atlas_code_quant.backtest.internal_gbm_simulator instead. "
    "Note: this module is NOT QuantConnect LEAN — it is the internal GBM "
    "synthetic simulator.",
    DeprecationWarning,
    stacklevel=2,
)

from atlas_code_quant.backtest.internal_gbm_simulator import (
    InternalGBMSimulator,
    LeanSimulator,
    SimConfig,
    TradeRecord,
)

__all__ = ["LeanSimulator", "SimConfig", "TradeRecord", "InternalGBMSimulator"]
```

Características:

- **Identidad estricta**: `lean_simulator.LeanSimulator is internal_gbm_simulator.LeanSimulator` → `True`.
- **`__module__`** de las clases públicas reporta `atlas_code_quant.backtest.internal_gbm_simulator` (la fuente real), no el shim.
- **`DeprecationWarning`** con `stacklevel=2` (el warning apunta al call-site del importador, no al shim).
- **No hay refactor funcional.** El shim sólo reexporta.

### 4.3 Imports antiguos soportados

Los siguientes imports siguen funcionando idénticamente tras F2 (con `DeprecationWarning`):

```python
from atlas_code_quant.backtest.lean_simulator import LeanSimulator
from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig, TradeRecord
from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig
import atlas_code_quant.backtest.lean_simulator as ls
ls.LeanSimulator(...)
```

Verificado en consumidores reales:

- `atlas_code_quant/learning/pattern_lab.py:497` (lazy import dentro de `retrain_from_lean_simulator`).
- `scripts/generate_lean_dataset.py:89`.

Imports recomendados desde F2:

```python
from atlas_code_quant.backtest.internal_gbm_simulator import (
    InternalGBMSimulator,
    SimConfig,
    TradeRecord,
)
```

---

## 5. Verificación / tests ejecutados

### 5.1 Tests F2 nuevos (archivo `tests/test_internal_gbm_simulator_aliasing.py`)

Cuatro tests, todos verdes:

```
atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py::test_canonical_path_exports               PASSED
atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py::test_deprecated_shim_exports_same_objects PASSED
atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py::test_class_module_is_canonical            PASSED
atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py::test_legacy_flag_advertises_internal_gbm  PASSED

============================== 4 passed in 0.40s ===============================
```

Verifican:

1. Ruta canónica `internal_gbm_simulator` exporta `InternalGBMSimulator`, `LeanSimulator`, `SimConfig`, `TradeRecord` y la identidad `InternalGBMSimulator is LeanSimulator`.
2. Ruta histórica `lean_simulator` reexporta los **mismos objetos** (identidad estricta) y emite `DeprecationWarning`.
3. `__module__` de las clases públicas apunta al fichero canónico.
4. `LEAN_SIMULATOR_IS_INTERNAL_GBM is True`.

### 5.2 Smoke imports adicionales

```bash
python -c "
from atlas_code_quant.backtest.internal_gbm_simulator import (
    InternalGBMSimulator, LeanSimulator, SimConfig, TradeRecord,
)
assert InternalGBMSimulator is LeanSimulator
"
# OK canonical: InternalGBMSimulator is LeanSimulator = True
# OK SimConfig.__module__       = atlas_code_quant.backtest.internal_gbm_simulator
# OK LeanSimulator.__module__   = atlas_code_quant.backtest.internal_gbm_simulator
# OK TradeRecord.__module__     = atlas_code_quant.backtest.internal_gbm_simulator
```

```bash
# Consumidor real pattern_lab (sin ejecutar el método; sólo importar el módulo)
python -c "import atlas_code_quant.learning.pattern_lab"
# OK atlas_code_quant.learning.pattern_lab importable

# Consumidor lazy: lean_simulator vía shim
python -c "
from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig
print(LeanSimulator.__name__, SimConfig.__name__)
"
# LeanSimulator SimConfig
# (con DeprecationWarning emitido)
```

### 5.3 Colección global de tests (no destructiva)

| Métrica | F1 (HEAD `31ef2c04`) | F2 (HEAD pendiente) | Delta |
|---|---|---|---|
| Tests recolectados | 964 | **968** | +4 (nuevos tests F2) |
| Errores de colección | 40 | **40** | 0 (mismos errores preexistentes por `sqlalchemy` ausente en sandbox) |

**No se introducen regresiones.** Los 4 tests nuevos son los del archivo F2; el resto del repo se mantiene exactamente igual.

### 5.4 Tests del simulador interno preexistentes

Búsqueda `grep -rn "LeanSimulator\|lean_simulator" atlas_code_quant/tests/` antes de F2 → **0 resultados**. No hay tests directos del simulador en el repo. Por tanto F2 cumple la cláusula de las instrucciones: "Si no hay tests directos, crea uno mínimo de import/alias, sin tocar comportamiento matemático."

---

## 6. Comandos ejecutados

```bash
# 1. Auditoría de consumidores
grep -rn "lean_simulator\|LeanSimulator" --include="*.py" .

# 2. Identificación de símbolos públicos
grep -nE "^(class|def) " atlas_code_quant/backtest/lean_simulator.py

# 3. Mover el archivo real (preservando historia git)
git rm atlas_code_quant/backtest/internal_gbm_simulator.py
git mv atlas_code_quant/backtest/lean_simulator.py \
       atlas_code_quant/backtest/internal_gbm_simulator.py

# 4. Editar docstring del archivo movido + añadir alias canónico al final.
# 5. Crear nuevo lean_simulator.py wrapper deprecated.
# 6. Ajustar comentario en config/legacy_flags.py.
# 7. Crear test mínimo y doc F2.

# 8. Smoke tests
python -c "from atlas_code_quant.backtest.internal_gbm_simulator import ..."
python -c "from atlas_code_quant.backtest.lean_simulator import ..."  # con DeprecationWarning
python -m pytest atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py -v

# 9. Colección no destructiva del repo entero
python -m pytest atlas_code_quant/tests --collect-only -q
# 968 tests collected, 40 errors

# 10. Commit atómico
git add -A
git commit -m "refactor: F2 rename internal GBM simulator and preserve lean compatibility shim"
```

---

## 7. Criterios de aceptación F2

- [x] `atlas_code_quant/backtest/internal_gbm_simulator.py` es el **único fichero canónico** del simulador (1245 LOC con docstring F2 + alias).
- [x] `atlas_code_quant/backtest/lean_simulator.py` es ahora **wrapper deprecated** (~70 LOC) que emite `DeprecationWarning` y reexporta.
- [x] `InternalGBMSimulator is LeanSimulator` → `True` (identidad estricta).
- [x] `lean_simulator.LeanSimulator is internal_gbm_simulator.LeanSimulator` → `True`.
- [x] Imports antiguos (`from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig, TradeRecord`) siguen funcionando.
- [x] Consumidores reales (`pattern_lab.py:497`, `scripts/generate_lean_dataset.py:89`) **no requieren cambio** y siguen funcionando.
- [x] No se ha modificado matemática del simulador (sin diff en helpers ni métodos de la clase).
- [x] No se ha tocado Radar / scanner / execution / Tradier / runtime / endpoints / locks.
- [x] 4 tests nuevos verdes (`test_internal_gbm_simulator_aliasing.py`).
- [x] Colección global: +4 tests nuevos, 0 regresiones.
- [x] Doc F2 generado.
- [x] Working tree limpio tras commit. Sin push.

---

## 8. Rollback exacto

Si F2 necesita revertirse:

```bash
git -C /home/user/workspace/atlas-core reset --hard 31ef2c04
# Vuelve al estado post-F1 (alias F1 lean→internal restaurado).
```

O alternativamente:

```bash
git -C /home/user/workspace/atlas-core revert <hash-F2> --no-edit
```

Notas sobre rollback:

- Como `git mv` mantiene historia, el revert restaurará el fichero original `lean_simulator.py` con su contenido completo y borrará el wrapper deprecated.
- El alias F1 `internal_gbm_simulator.py` (35 líneas, re-export hacia lean) será restaurado.
- Los 4 tests nuevos serán removidos.
- Consumidores `pattern_lab.py` y `scripts/generate_lean_dataset.py` siguen funcionando idénticamente antes y después del revert (ya funcionaban con el path histórico).

---

## 9. Cierre F2 — STOP

- **Working tree:** limpio tras commit.
- **Push:** no realizado.
- **Doc F2:** generada (este archivo).
- **Próximo paso:** esperar aprobación humana **explícita** antes de F3.
