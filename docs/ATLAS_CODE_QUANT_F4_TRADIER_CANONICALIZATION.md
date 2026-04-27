# F4 — Canonicalización Tradier

> **Fase:** F4 (Atlas Code Quant Refactor)
> **Estado:** completado, pendiente aprobación humana antes de F5+
> **Commit:** `refactor: F4 document Tradier canonical implementation and mark phase1 broker as legacy`
> **Push:** NO autorizado en F4 (commit local únicamente)
> **Riesgo runtime:** nulo — sólo se añaden docstrings, un README y dos constantes documentales.

---

## 1. Objetivo

Identificar y declarar la implementación canónica de Tradier en `atlas_code_quant`, y marcar como LEGACY/PHASE1 la implementación paralela en `atlas_options_brain_fase1`, **sin** cambiar imports, lógica de envío de órdenes ni routing efectivo.

F4 **NO**:

* cambia firmas, lógica ni semántica de ningún módulo Tradier;
* añade endpoints;
* mueve archivos;
* toca `broker_router`, locks, risk, scanner, intake, Radar;
* unifica los stacks (eso queda fuera de F4).

## 2. Reglas duras respetadas

* ✅ Sólo se modifican docstrings + un README nuevo + flags documentales en `config/legacy_flags.py` + un test nuevo + esta doc.
* ✅ NO se cambian imports existentes.
* ✅ NO se cambia lógica de envío de órdenes Tradier.
* ✅ NO se tocan `broker_router`, risk, locks, Radar, scanner, intake.
* ✅ NO se añaden nuevos endpoints.
* ✅ NO push.

## 3. Inventario Tradier (estado pre-F4 / post-F4)

### 3.1 Stack canónico — `atlas_code_quant/execution/`

| Archivo | Rol | LOC |
|---|---|---|
| `atlas_code_quant/execution/tradier_execution.py` | Routing de órdenes equity / option / multileg / combo. Entry point `route_order_to_tradier`. | 589 (+ docstring F4) |
| `atlas_code_quant/execution/tradier_controls.py` | Resolución de cuenta + PDT + cache de sesión (`TradierAccountSession`, `resolve_account_session`, `check_pdt_status`). | 342 |
| `atlas_code_quant/execution/tradier_pdt_ledger.py` | Ledger local intraday PDT (`record_live_order_intent`, `count_intraday_day_trades`). | 147 |
| `atlas_code_quant/pipeline/tradier_stream.py` | Streaming WS / quotes / options chain. **No** envía órdenes. | 397 |

Símbolos públicos clave de `tradier_execution.py` (verificados por test):

```
TradierOrderBlocked
should_route_to_tradier
is_opening_order
build_tradier_order_payload
route_order_to_tradier
```

### 3.2 Stack legacy/PHASE1 — `atlas_options_brain_fase1/atlas_options_brain/`

| Archivo | Rol | LOC |
|---|---|---|
| `broker/tradier_executor.py` | Ejecutor HTTP multileg sandbox. `TradierOrderExecutor`, `TradierMultilegType`. | 213 (+ docstring F4) |
| `broker/tradier_live.py` | Tipos / builder / sink sin red. `LiveOrder`, `LiveOrderLeg`, `TradierOrderBuilder`, `TradierLiveExecutionSink`. | 186 (+ docstring F4) |
| `broker/__init__.py` | Re-export de los anteriores. | 18 (+ docstring F4) |
| `providers/tradier_provider.py` | Cliente de **datos** Tradier (cadena de opciones, no envío de órdenes). | 123 (+ docstring F4) |

### 3.3 Scripts auxiliares en raíz del repo

| Archivo | Rol | Estado F4 |
|---|---|---|
| `chk_tradier.py` | Script ad-hoc de verificación de órdenes en sandbox. Token y cuenta hardcodeados en sandbox. | No tocado en F4 (script de operación local; no es ruta canónica). |
| `close_tradier_positions.py` | Script ad-hoc de cierre de posiciones en sandbox. | No tocado en F4. |

> Estos scripts no son código de runtime ni participan del routing institucional. Si llegasen a migrarse, deberían usar `atlas_code_quant.execution.tradier_execution`. F4 no los modifica para preservar reglas duras (no tocar lógica de envío).

## 4. Rutas reales en runtime — qué llama a quién

### 4.1 Consumidores del stack **CANÓNICO** (sin cambios en F4)

```
atlas_code_quant/api/main.py                (rutas /orders, etc.)
atlas_code_quant/api/decorators.py
atlas_code_quant/execution/broker_router.py (línea 18 import directo)
atlas_code_quant/execution/signal_executor.py
atlas_code_quant/operations/operation_center.py
atlas_code_quant/operations/auton_executor.py
atlas_code_quant/production/live_activation.py
atlas_code_quant/start_paper_trading.py
atlas_code_quant/tests/test_tradier_execution_fallback.py
scripts/options_preflight_cleanup.py
```

`BrokerRouter` rutea `equity_stock` / `equity_etf` / `index_option` → `route_order_to_tradier` (línea 59 de `broker_router.py`).

### 4.2 Consumidores del stack **PHASE1** (sin cambios en F4)

```
atlas/core/options_live.py                                          (sandbox-only)
atlas/tests/test_options_live_executor.py
atlas_options_brain_fase1/atlas_options_brain/__init__.py
atlas_options_brain_fase1/atlas_options_brain/broker/__init__.py
atlas_options_brain_fase1/atlas_options_brain/tests/test_tradier_executor.py
atlas_options_brain_fase1/atlas_options_brain/tests/test_tradier_live.py
```

`atlas/core/options_live.py` (`OptionsLiveExecutor`) rechaza explícitamente ejecutores con `sandbox=False` o `allow_production=True`. Por tanto: el stack PHASE1 **no** participa en producción; sólo en sandbox de Atlas Options Brain y en tests de phase1.

## 5. Por qué cada cosa

### 5.1 Por qué `atlas_code_quant/execution/` es canónico

1. **Único stack** al que llegan rutas reales de envío de órdenes.
2. Implementa **locks de seguridad** del proyecto: `paper_only`, `full_live_globally_locked`, `ATLAS_FORCE_LIVE_PREVIEW`, dry-run defaults.
3. Integra **PDT controls** y ledger local.
4. Implementa **reconciliación post-timeout** (polling + stream).
5. Cubierto por `test_tradier_execution_fallback.py` y por las tests de `operation_center` / `production_guard`.

### 5.2 Por qué PHASE1 es legacy

1. Snapshot de la fase de entrenamiento de Atlas Options Brain.
2. Único consumidor en el repo principal: `atlas/core/options_live.py` con `sandbox=True` forzado.
3. **No** integrado con locks `atlas_code_quant.config.settings`, ni con ledger PDT, ni con reconciliación.
4. Se conserva sólo para tests congelados de phase1 y referencia histórica de la API multileg.

## 6. Cambios introducidos por F4

### 6.1 Marcado canónico

* `atlas_code_quant/execution/tradier_execution.py`: docstring extendido con marca `.. canonical:: F4`, lista de consumidores y referencia al README.
* `atlas_code_quant/execution/README_TRADIER.md` **(nuevo)**: declaración explícita del stack canónico, justificación, lista de consumidores, reglas duras y plan de migración futuro.

Extracto del docstring nuevo en `tradier_execution.py`:

```text
.. canonical:: F4
    Esta es la implementación **CANÓNICA** de Tradier para Atlas Code Quant.

    Stack: ``atlas_code_quant.execution`` (este módulo + ``tradier_controls`` +
    ``tradier_pdt_ledger``).
    ...
    Implementación paralela legacy (NO usar para routing de producción,
    sólo training/PHASE1):
        * ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_executor``
        * ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_live``

    F4 NO cambia firmas, lógica de envio de órdenes, locks ni routing efectivo.
```

### 6.2 Marcado LEGACY/PHASE1

Cuatro archivos PHASE1 reciben la marca `.. legacy:: F4 PHASE1` en su docstring **a nivel módulo**, sin tocar lógica:

* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py`
* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py`
* `atlas_options_brain_fase1/atlas_options_brain/broker/__init__.py`
* `atlas_options_brain_fase1/atlas_options_brain/providers/tradier_provider.py`

Extracto del docstring nuevo en `broker/tradier_executor.py`:

```text
.. legacy:: F4 PHASE1
    Este módulo pertenece al **stack legacy/PHASE1** de Atlas Options Brain
    (``atlas_options_brain_fase1``). **NO** es la implementación Tradier
    canónica de Atlas Code Quant.

    * Stack canónico (runtime): ``atlas_code_quant.execution.tradier_execution``
        (+ ``tradier_controls``, ``tradier_pdt_ledger``).
    * Este archivo se conserva como referencia histórica de la API multileg
        de Tradier y para soportar tests congelados de phase1
        (...test_tradier_executor.py).
    * Su único consumidor en el repo principal es
        ``atlas/core/options_live.py``, que opera estrictamente sobre sandbox.
    * NO está integrado con los locks de seguridad de
        ``atlas_code_quant.config.settings``, ni con el ledger PDT, ni con la
        reconciliación post-timeout del stack canónico.
    * NO importar desde código de producción de ``atlas_code_quant``.
    * F4 marca este módulo como LEGACY/PHASE1 sin cambiar firmas ni lógica
        interna.
```

### 6.3 Flags documentales (`atlas_code_quant/config/legacy_flags.py`)

```python
ATLAS_TRADIER_CANONICAL_STACK: str = "atlas_code_quant"
ATLAS_TRADIER_PHASE1_LEGACY_STACK: str = "atlas_options_brain_fase1"
```

* **Documentales** en F4: ningún módulo de runtime las consulta.
* Sirven como contrato público para auditoría.
* No se usan como condicionales de ruta.

### 6.4 Test estructural (`atlas_code_quant/tests/test_tradier_canonical_surface.py`)

Nuevo test (21 casos) que verifica:

* Existencia física de los 3 módulos canónicos y del README.
* Marca `.. canonical:: F4` en `tradier_execution.py` y "canónica" en el README.
* Superficie pública canónica intacta: `TradierOrderBlocked`, `should_route_to_tradier`, `is_opening_order`, `build_tradier_order_payload`, `route_order_to_tradier`, `TradierAccountSession`, `record_live_order_intent`, `count_intraday_day_trades`, etc.
* PHASE1 sigue presente físicamente (`tradier_executor.py`, `tradier_live.py`, `broker/__init__.py`, `tradier_provider.py`).
* PHASE1 marcado con `.. legacy:: F4 PHASE1` y referenciando al stack canónico.
* Símbolos públicos PHASE1 intactos: `TradierOrderExecutor`, `TradierMultilegType`, `LiveOrder`, `LiveOrderLeg`, `TradierLiveExecutionSink`, `TradierOrderBuilder`.
* Flags F4 publicadas con valores correctos en `legacy_flags.__all__`.

> Se usa AST en lugar de import dinámico porque ambos stacks tienen dependencias preexistentes faltantes en el sandbox (`backtesting` para canónico, `atlas_options_brain` para PHASE1). Esto es ortogonal a F4.

## 7. Verificaciones de imports

| Import | Resultado | Causa |
|---|---|---|
| `import atlas_code_quant.execution.tradier_execution` | ❌ falla preexistente | `ModuleNotFoundError: 'backtesting'` (línea 50). **No introducido por F4**; mismo error que en F0/F1/F2/F3. |
| `import atlas_options_brain_fase1.atlas_options_brain.broker.tradier_executor` | ❌ falla preexistente | `ModuleNotFoundError: 'atlas_options_brain'` (path alias no configurado en sandbox). **No introducido por F4**. |
| `import atlas_code_quant.config.legacy_flags` | ✅ OK |
| `import atlas_code_quant` | ✅ OK |

Los dos errores `ModuleNotFoundError` son los mismos que existen desde F0 (40 errores de colección preexistentes). F4 no los empeora ni los altera.

## 8. Resultado de pytest

* `pytest atlas_code_quant/tests/test_tradier_canonical_surface.py -q` → **21 passed** en 0.14 s.
* `pytest atlas_code_quant/tests/test_scanner_deprecation_headers.py atlas_code_quant/tests/test_internal_gbm_simulator_aliasing.py atlas_code_quant/tests/test_tradier_canonical_surface.py -q` → **90 passed** (todos los tests añadidos en F2/F3/F4 conviven sin regresión).
* `pytest atlas_code_quant/tests --collect-only -q` → **1054 tests collected, 40 errors** (preexistentes). Pre-F4 era 1033. Delta = +21 tests del nuevo archivo F4.

## 9. Criterio de aceptación F4

1. ✅ Inventario Tradier completo y documentado.
2. ✅ `atlas_code_quant/execution/tradier_execution.py` declarado canónico (docstring `.. canonical:: F4`).
3. ✅ `atlas_code_quant/execution/README_TRADIER.md` creado con la declaración canónica.
4. ✅ Cuatro archivos PHASE1 etiquetados `.. legacy:: F4 PHASE1` sin cambios de lógica.
5. ✅ Flags `ATLAS_TRADIER_CANONICAL_STACK` y `ATLAS_TRADIER_PHASE1_LEGACY_STACK` añadidas en `legacy_flags.py` con valores correctos y exportadas en `__all__`.
6. ✅ Test nuevo `test_tradier_canonical_surface.py` con 21/21 passed.
7. ✅ Imports existentes intactos (los `ModuleNotFoundError` son preexistentes y no aparecen ni desaparecen por F4).
8. ✅ Tests pre-F4 sin regresión (1033 → 1054, mismos 40 errores preexistentes).
9. ✅ Commit atómico F4 sin push.

## 10. Plan de rollback

F4 no toca lógica. Reversión limpia:

```bash
git -C /home/user/workspace/atlas-core revert <hash-commit-F4>
```

Esto retira los docstrings F4, el README canónico, las flags `ATLAS_TRADIER_*`, el test estructural y esta doc. No quedan efectos laterales en runtime.

## 11. Migración futura (NO implementada en F4)

Cualquier consolidación posterior pasará por una fase específica que:

1. Identifique paridad funcional (multileg, locks, PDT, reconciliación).
2. Extraiga la API multileg compartida si aplica, dejando phase1 como wrapper deprecated análogo a `lean_simulator.py` (F2).
3. Reemplace o adapte `atlas/core/options_live.py` para apoyarse en el stack canónico cuando proceda (manteniendo `sandbox=True` forzado).
4. Añada tests de paridad antes de retirar phase1.
5. Sólo entonces actualice `ATLAS_TRADIER_PHASE1_LEGACY_STACK` o introduzca una flag de cutover.

## 12. Referencias

* `atlas_code_quant/execution/tradier_execution.py` (docstring `.. canonical:: F4`, líneas 1–39)
* `atlas_code_quant/execution/README_TRADIER.md` (nuevo)
* `atlas_code_quant/config/legacy_flags.py` (flags F4, líneas ~64–106)
* `atlas_code_quant/tests/test_tradier_canonical_surface.py` (21 tests)
* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py` (docstring `.. legacy:: F4 PHASE1`)
* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py` (docstring `.. legacy:: F4 PHASE1`)
* `atlas_options_brain_fase1/atlas_options_brain/broker/__init__.py` (docstring `.. legacy:: F4 PHASE1`)
* `atlas_options_brain_fase1/atlas_options_brain/providers/tradier_provider.py` (docstring `.. legacy:: F4 PHASE1`)
* `docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md` (F0)
* `docs/ATLAS_CODE_QUANT_F1_REORG.md` (F1)
* `docs/ATLAS_CODE_QUANT_F2_INTERNAL_SIMULATOR_RENAME.md` (F2)
* `docs/ATLAS_CODE_QUANT_F3_SCANNER_DEPRECATION.md` (F3)
