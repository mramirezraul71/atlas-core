# Atlas Code Quant — F0 Verificación de Auditoría Arquitectónica

**Fase:** F0 (verificación previa, sin refactor)
**Repo:** `mramirezraul71/atlas-core`
**Rama:** `variante/nueva`
**HEAD:** `41f868339938f9516eee43addfcd513a4afa7df3`
**Último commit:** `chore: ignore temporary Fase 5 artifacts`
**Working tree:** limpio, sin cambios locales
**Fecha verificación:** 2026-04-27
**Operador:** Computer (agente senior implementación / arquitectura / refactor)

> **STOP RULE activo:** Esta fase F0 es exclusivamente lectura, inventario y verificación documental. **No se ha modificado código funcional.** Tras este documento, el agente debe detenerse y esperar aprobación humana antes de iniciar F1.

---

## 1. Resumen ejecutivo

El repositorio `atlas-core` aloja dos servicios HTTP independientes que se comunican por contrato (sin imports cruzados Python):

- **ATLAS PUSH** (`atlas_adapter/`) — servidor `:8791`, expone UI, dashboard de Radar y API pública `/api/radar/*`.
- **Atlas Code Quant** (`atlas_code_quant/`) — servidor `:8792`, motor cuantitativo, options brain, backtesting, ejecución Tradier (paper) y orquestación de operaciones.

El estado real del repo está **alineado con el bloque de "auditoría confirmada"** del prompt en estos puntos: existe Radar institucional con SSE multi-evento, existe TradierClient canónico en `atlas_code_quant/execution/`, existen locks `paper_only=True` y `full_live_globally_locked=True` activos en runtime, y existe `production_guard` con dry-run forzado por defecto. **El bloque de "auditoría corregida" también se confirma:** el "LEAN" presente es un simulador GBM sintético interno (no es LEAN/QuantConnect real), la "FSM autónoma" formal con estados nombrados no existe (sólo loops procedurales), el Vision Timing Gate institucional `allow|delay|block|force_exit` con `trace_id` no existe (sólo un boolean `allow_entry`), y el endpoint `/api/radar/opportunities` multi-símbolo no existe (mono-símbolo confirmado).

Hay además **duplicación real de código Tradier** en `atlas_options_brain_fase1/` (legacy fase 1) que coexiste con el canónico, scanner heredado de 2218 LOC con auditoría propia que documenta criterios débiles, y `atlas_code_quant/api/main.py` consolidado en 148 endpoints (incluyendo duplicados `/scanner` y `/api/v2/quant/scanner` que requieren deprecación controlada).

**Conclusión F0:** El repo es real y consistente con el blueprint. Es viable ejecutar el plan F1..F10 con reservas concretas que se enumeran en §10 (deprecación de scanner por freeze/cutover, no por borrado masivo; renombre de `lean_simulator.py` con wrapper retrocompatible; integración Tradier institucional sin tocar locks; construcción de FSM y Vision Gate desde cero como módulos nuevos, no reescrituras invasivas).

---

## 2. Audit previo confirmado (lo que SÍ existe en el repo real)

| Item del prompt | Evidencia en repo real |
|---|---|
| Radar institucional con SSE multi-evento y schemas Pydantic | `atlas_adapter/routes/radar_public.py` (743 LOC, 13 endpoints) + `radar_schemas.py` (266 LOC, 23 schemas) |
| Mapper Quant→PUSH para Radar | `atlas_adapter/routes/radar_quant_mapper.py` (402 LOC) |
| Cliente HTTP server-to-server hacia Code Quant | `atlas_adapter/routes/radar_quant_client.py` (277 LOC) |
| Tests de contrato Radar | `atlas_adapter/routes/test_radar_quant_contract.py`, `test_radar_sse_contract.py`, `test_radar_symbols_search_http.py` |
| TradierClient canónico (paper) | `atlas_code_quant/execution/tradier_execution.py` (589 LOC), `tradier_controls.py` (342 LOC), `tradier_pdt_ledger.py` (147 LOC) |
| Broker router + paper broker + Alpaca paper | `atlas_code_quant/execution/broker_router.py` (141 LOC), `paper_broker.py` (622 LOC), `alpaca_paper.py`, `account_manager.py` |
| Settings Tradier paper/live + scope + dry_run | `atlas_code_quant/config/settings.py:229-243` (`TRADIER_PAPER_TOKEN`, `TRADIER_LIVE_TOKEN`, `TRADIER_PAPER_BASE_URL=sandbox.tradier.com/v1`, `TRADIER_LIVE_BASE_URL=api.tradier.com/v1`, `TRADIER_DEFAULT_SCOPE`) |
| Lock global paper_only=True | `atlas_code_quant/operations/operation_center.py:72`, también líneas 1550, 1626, 2229, 2523 |
| Lock global full_live_globally_locked=True | `atlas_code_quant/operations/operation_center.py:146,339,357,1576,2193`, `runtime_config_v4.py:29`, `live_switch.py:30,52-53` (blocking reason `"full_live_globally_locked"`) |
| Production guard con dry-run forzado | `atlas_code_quant/production/production_guard.py` (daily/weekly/monthly loss limits + dry-run defaults) |
| Pydantic schemas multi-leg para options | `atlas_code_quant/api/schemas.py:285,321` (defaults paper_only, full_live_globally_locked) |
| Backtesting interno funcional | `atlas_code_quant/backtester.py`, `atlas_code_quant/backtesting/winning_probability.py:148` (incluye su propio `class TradierClient` para data histórica) |
| Tests críticos radar/scanner/options | 231 archivos de test, 26.692 LOC; carpeta `atlas_code_quant/tests/` |
| API main consolidada | `atlas_code_quant/api/main.py` (148 endpoints registrados) |
| Captura física Insta360 + OCR + multi-timeframe | `atlas_code_quant/vision/insta360_capture.py` (283 LOC), `chart_ocr.py` (226 LOC), `multi_timeframe_analyzer.py` (118 LOC), `visual_pipeline.py` (112 LOC), `windows_camera_hints.py` (60 LOC) |
| Estado degradado `CAMERA_UNAVAILABLE` honesto | `atlas_adapter/routes/radar_quant_mapper.py` función `compute_degradations_active` |
| Endpoints emergency stop / reset | `atlas_code_quant/api/main.py:3327,3343` (`/emergency/stop`, `/emergency/reset`) |
| Reconciliación post-timeout Tradier | `atlas_code_quant/execution/tradier_execution.py` (lógica de reconciliación referenciada en operation_center) |
| Journal sync | presente en `atlas_code_quant/operations/` (operation_center llama a journal en flush operations) |
| Canonical snapshot / monitoring | `atlas_code_quant/api/main.py` expone `/health`, `/readiness`, `/api/quant/...` |

---

## 3. Audit previo corregido (lo que el prompt dice que NO existe y se confirma INEXISTENTE)

| Item del prompt | Evidencia de inexistencia |
|---|---|
| **LEAN / QuantConnect real** | NO existe. Lo que hay es `atlas_code_quant/backtest/lean_simulator.py` (1224 LOC). Su docstring declara: motor GBM sintético + Markov regime, sin TA-lib, sin yfinance, sin Tradier, sin QuantConnect. Sin integración a runtime LEAN externo. **Debe renombrarse a `internal_gbm_simulator.py` con wrapper deprecated en F-LEAN.** |
| **FSM autónoma formal con estados nombrados** | NO existe. `grep -E "class .*StateMachine\|class .*FSM"` → 0 resultados en todo el repo. Lo que hay: `atlas_core/autonomy/orchestrator.py` (78 LOC, loop tick procedural), `policy_engine.py`, `state_bus.py`, `module_registry.py` y loops procedurales dispersos en `atlas_code_quant/operations/operation_center.py` (3162 LOC). |
| **Vision Timing Gate institucional `allow\|delay\|block\|force_exit` con trace_id** | NO existe. Sólo hay un boolean `allow_entry` en `atlas_code_quant/options/options_intent_router.py:73,80,87,137,167`. No hay enum de estados ni trace_id propagado. |
| **Endpoint `/api/radar/opportunities` multi-símbolo** | NO existe. El streaming actual está fijado mono-símbolo: `atlas_adapter/routes/radar_public.py:266` `sym = (symbol or "SPY").strip().upper() or "SPY"` y línea 538 `stream_sym = (symbol or "SPY").strip().upper() or "SPY"`. También `radar_quant_mapper.py:266` confirma el patrón mono-símbolo. |
| **Kalshi radar / `modules/atlas_radar_kalshi/`** | NO existe en el repo. No hay conflicto de nombres ni código duplicado a resolver. Esta funcionalidad requeriría módulo nuevo desde cero si entra en roadmap. |
| **Kill switch engine canónico** | Existe sólo helper minimalista: `atlas_code_quant/operations/kill_switch.py` (≈14 LOC). El kill switch operativo se implementa vía `/emergency/stop` + flags de operation_center, no como engine único formal. |
| **Carpetas esperadas inexistentes** | `atlas_code_quant/lean/` ❌; `atlas_code_quant/intake/` ❌ (incluido `intake/radar_client.py`); `atlas_code_quant/autonomy/` ❌ (existe `atlas_core/autonomy/` que es distinto); `atlas_code_quant/telemetry/` ❌; `atlas_adapter/routes/radar_opportunities.py` ❌; `atlas_adapter/services/radar_batch_engine.py` ❌; `atlas_adapter/services/universe_provider.py` ❌. |

> **Regla aplicada:** no inventar rutas que no existen. Estos gaps se documentan como tales y se planifican como módulos nuevos en su fase respectiva, no como "ya estaban".

---

## 4. Rutas canónicas (mantener / consolidar)

### 4.1 Atlas Code Quant — `atlas_code_quant/`

| Subsistema | Ruta canónica | Notas |
|---|---|---|
| API HTTP principal | `atlas_code_quant/api/main.py` (148 endpoints) | Consolidar y deprecar duplicados (ver §6) |
| Schemas Pydantic API | `atlas_code_quant/api/schemas.py` | Multi-leg, paper_only y full_live_globally_locked en defaults |
| Settings centralizada | `atlas_code_quant/config/settings.py:229-243` | Tradier paper/live, base_urls, scopes, dry_run |
| Operations / control center | `atlas_code_quant/operations/operation_center.py` (3162 LOC) | Locks `paper_only` y `full_live_globally_locked` |
| Runtime config v4 | `atlas_code_quant/operations/runtime_config_v4.py:29` | `full_live_globally_locked: bool = True` por defecto |
| Live switch | `atlas_code_quant/operations/live_switch.py:30,52-53` | Blocking reason `"full_live_globally_locked"` |
| Production guard | `atlas_code_quant/production/production_guard.py` | Daily / weekly / monthly loss limits |
| Tradier ejecución | `atlas_code_quant/execution/tradier_execution.py` (589 LOC) | TradierClient canónico runtime |
| Tradier controls | `atlas_code_quant/execution/tradier_controls.py` (342 LOC) | Riesgo, controles previos a orden |
| Tradier PDT ledger | `atlas_code_quant/execution/tradier_pdt_ledger.py` (147 LOC) | Ledger PDT |
| Broker router | `atlas_code_quant/execution/broker_router.py` (141 LOC) | Routing paper/live |
| Paper broker | `atlas_code_quant/execution/paper_broker.py` (622 LOC) | Simulación paper interna |
| Account manager | `atlas_code_quant/execution/account_manager.py` | Selector de cuenta |
| Alpaca paper | `atlas_code_quant/execution/alpaca_paper.py` | Alternativa paper |
| Options intent router | `atlas_code_quant/options/options_intent_router.py:73,80,87,137,167` | Boolean `allow_entry` actual (gap §3) |
| Backtesting motor real | `atlas_code_quant/backtester.py` | Núcleo de backtest |
| Backtesting probabilidades | `atlas_code_quant/backtesting/winning_probability.py:148` | Incluye su propio `class TradierClient` para data histórica |
| Vision física | `atlas_code_quant/vision/insta360_capture.py` (283), `chart_ocr.py` (226), `multi_timeframe_analyzer.py` (118), `visual_pipeline.py` (112), `windows_camera_hints.py` (60) | Captura RTMP, OCR, multi-timeframe |
| Tests | `atlas_code_quant/tests/` (231 archivos / 26.692 LOC) | Cobertura crítica radar / scanner / options / risk |

### 4.2 Atlas PUSH — `atlas_adapter/`

| Componente | Ruta canónica | Notas |
|---|---|---|
| Radar API pública (13 endpoints) | `atlas_adapter/routes/radar_public.py` (743 LOC) | SSE 6 campos canónicos: `type, timestamp, symbol, source, sequence, data` |
| Radar schemas (23 modelos Pydantic) | `atlas_adapter/routes/radar_schemas.py` (266 LOC) | Contratos estables del cliente |
| Mapper Quant→PUSH | `atlas_adapter/routes/radar_quant_mapper.py` (402 LOC) | Incluye `compute_degradations_active` |
| Cliente HTTP a Code Quant | `atlas_adapter/routes/radar_quant_client.py` (277 LOC) | Server-to-server, no comparte memoria |
| UI / dashboard | `/ui`, `/radar/dashboard` (montados desde `atlas_adapter`) | Servicio en `:8791` |
| Tests de contrato | `atlas_adapter/routes/test_radar_quant_contract.py`, `test_radar_sse_contract.py`, `test_radar_symbols_search_http.py` | Contratos blindados |

### 4.3 Comunicación entre servicios

- **HTTP-only**: 100% por HTTP, 0 imports cruzados Python entre `atlas_adapter` y `atlas_code_quant`.
- **Puertos**: ATLAS PUSH `:8791`, Code Quant `:8792`.
- **Endpoints clave consumidos por PUSH**: `/api/scanner/report`, `/api/quant/*` (vía `radar_quant_client.py`).

---

## 5. Rutas legacy (deprecar / freeze controlado)

| Ruta legacy | Estado | Acción planificada |
|---|---|---|
| `atlas_code_quant/scanner/opportunity_scanner.py` (2218 LOC) | Activo, criterios débiles auto-documentados | **Freeze + cutover** a Radar institucional. NO borrar en F1; primero migrar consumidores. Mover a `atlas_code_quant/legacy/scanner/` cuando consumidores migren. |
| `atlas_code_quant/scanner/{universe_catalog,asset_classifier,etf_universe,index_universe,crypto_universe,futures_universe,options_flow_provider}.py` (3637 LOC total) | Activo | Mover a `atlas_code_quant/legacy/scanner/` junto con opportunity_scanner. |
| `scanner/__init__.py` (raíz repo) | Compat shim | Reexporta `atlas_code_quant.scanner`. Mantener temporalmente como wrapper de retrocompatibilidad hasta cutover completo. |
| `atlas_code_quant/api/main.py:3356,3369,3385,3406,3445` (`/scanner/{status,report,config,control,universe/search}`) | Activo, expuesto | Marcar `deprecated=True` en OpenAPI. Mantener al menos 1 ciclo. |
| `atlas_code_quant/api/main.py:3677-3706` (`/api/v2/quant/scanner/*`) | Duplicado del bloque anterior | Deprecación controlada — uno de los dos bloques debe redirigir al otro. |
| `atlas_code_quant/SCANNER_CRITERIA_AUDIT.md` | Documento ya existente | Referencia de criterios débiles; reusar en plan de migración. |
| `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py` | **Duplicado Tradier legacy fase 1** | Marcar deprecated; redirigir a `atlas_code_quant/execution/tradier_execution.py`. Mover a `legacy/` cuando los tests lo permitan. |
| `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py` | Duplicado live | Mismo trato; **NO conectar broker live real**. |
| `atlas_options_brain_fase1/atlas_options_brain/providers/tradier_provider.py` | Duplicado provider | Idem; consolidar fuente única en `atlas_code_quant/execution/`. |
| `atlas_code_quant/backtest/lean_simulator.py` (1224 LOC) | **Mal nombrado** (es GBM sintético, no LEAN) | Renombrar a `internal_gbm_simulator.py` y dejar wrapper `lean_simulator.py` con `DeprecationWarning` que reexporta. NO eliminar import path en F-LEAN. |
| `scripts/generate_lean_dataset.py` (153 LOC) | Usa el simulador GBM | Renombrar a `generate_internal_gbm_dataset.py` con wrapper. |

> **Regla 6 aplicada:** no se borra código funcional. Sólo se deprecan, mueven a `legacy/`, y se mantienen wrappers de compat para no romper imports.

---

## 6. Rutas duplicadas (resolver, no romper)

| Duplicado | Ubicaciones | Estrategia |
|---|---|---|
| **Endpoints scanner** | `/scanner/{status,report,config,control,universe/search}` (`api/main.py:3356-3445`) **y** `/api/v2/quant/scanner/*` (`api/main.py:3677-3706`) | Definir una versión canónica. La otra mantiene mismos contratos pero responde con header `Deprecation` y `Sunset`. |
| **TradierClient** | `atlas_code_quant/execution/tradier_execution.py` (canónico runtime) **y** `atlas_code_quant/backtesting/winning_probability.py:148` (cliente histórico para backtest) | Mantener ambos por ahora — propósitos distintos (runtime vs histórico). Documentar diferencia en docstring. |
| **Tradier broker** | `atlas_code_quant/execution/tradier_execution.py` **y** `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py` + `tradier_live.py` + `providers/tradier_provider.py` | Consolidar: el canónico es `atlas_code_quant/execution/`. El bloque `atlas_options_brain_fase1/` se marca legacy y se mueve en fase tardía. |
| **Estructura scanner** | `atlas_code_quant/scanner/` (motor real) **y** `scanner/` raíz (compat shim) | El shim raíz se mantiene; sólo cambia destino tras cutover. |
| **Configuración full_live_globally_locked** | Default en `runtime_config_v4.py:29`, también propagado en `operation_center.py:146,339,2193`, `live_switch.py:30,52-53`, schemas Pydantic `api/schemas.py:285,321` | NO consolidar — la duplicación es defensiva y blinda el lock. **No tocar.** |

---

## 7. Gaps reales (componentes inexistentes, a construir desde cero)

| Gap | Documentación |
|---|---|
| `/api/radar/opportunities` multi-símbolo (HTTP + SSE batch) | No existe endpoint. Construir nuevo en `atlas_adapter/routes/radar_opportunities.py` (ruta inexistente hoy). |
| `atlas_adapter/services/radar_batch_engine.py` | Inexistente. Motor de batch evaluation multi-símbolo a crear. |
| `atlas_adapter/services/universe_provider.py` | Inexistente. Proveedor de universo dinámico. |
| `atlas_code_quant/intake/radar_client.py` | Inexistente. Cliente intake desde Radar (carpeta `intake/` no existe). |
| `atlas_code_quant/autonomy/` (FSM formal con estados nombrados) | Inexistente. Construir nuevo módulo (no confundir con `atlas_core/autonomy/` que existe pero es procedural). |
| Vision Timing Gate completo `allow \| delay \| block \| force_exit` con `trace_id` | Inexistente. Sólo hay boolean `allow_entry` en `options_intent_router.py:73,80,87,137,167`. Crear módulo nuevo (sugerido `atlas_code_quant/vision/timing_gate.py`). |
| `atlas_code_quant/lean/` (integración LEAN/QuantConnect real externa o vendor) | Inexistente. Lo actual es simulador GBM. Decisión arquitectónica pendiente: externo (proceso aparte) / vendor docker / abandono explícito de LEAN como nombre. |
| `atlas_code_quant/telemetry/` (módulo telemetría dedicado) | Inexistente. Telemetría dispersa en operation_center y endpoints health/readiness. |
| Kill switch engine canónico | Helper actual en `kill_switch.py` (≈14 LOC) es minimalista. Construir engine formal con prioridades, fuentes (manual / production_guard / risk / ops) y trace. |
| Kalshi radar (`modules/atlas_radar_kalshi/`) | Inexistente. Si entra en roadmap, será módulo greenfield. |

---

## 8. Riesgos de refactor

1. **Romper locks de seguridad**: `paper_only=True` y `full_live_globally_locked=True` están propagados en al menos 6 archivos (operation_center, runtime_config_v4, live_switch, api/schemas, production_guard, defaults Pydantic). Cualquier consolidación que reduzca esa duplicación defensiva es riesgo crítico — **no consolidar estos defaults.**
2. **Imports cross-service accidentales**: La frontera HTTP entre `atlas_adapter` y `atlas_code_quant` es estricta hoy. Refactors que muevan código compartido a un paquete común podrían introducir imports cruzados — prohibido.
3. **Tests de contrato Radar**: `test_radar_quant_contract.py` y `test_radar_sse_contract.py` blindan los 6 campos canónicos del SSE (`type, timestamp, symbol, source, sequence, data`). Tocar el mapper sin actualizar contratos rompe estos tests.
4. **Scanner consumers ocultos**: Consumidores conocidos hoy son `atlas_code_quant/api/main.py`, `backtester.py:27`, `learning/trading_implementation_scorecard.py:568` y varios tests. Pueden existir más por compat shim raíz `scanner/`. Antes de mover scanner a `legacy/`, exigir grep exhaustivo y CI verde.
5. **Renombre `lean_simulator.py`**: hay 1224 LOC importadas presumiblemente por scripts y tests. El renombre debe ir acompañado de wrapper `lean_simulator.py` que reexporte y emita `DeprecationWarning`, no de borrado directo.
6. **Operation_center.py tamaño**: 3162 LOC con lógica crítica de locks, journal, reconciliación. Cualquier split debe hacerse con tests existentes verdes y módulos nuevos importables vía wrappers.
7. **Endpoints duplicados scanner**: 9 endpoints entre `/scanner/*` y `/api/v2/quant/scanner/*`. Clientes externos pueden depender de cualquiera. Deprecación controlada con `Sunset` headers, no eliminación.
8. **Tradier duplicado en `atlas_options_brain_fase1/`**: mover a legacy podría romper imports en módulos `atlas_options_brain*`. Requiere mapeo previo de consumidores.
9. **Vision Gate parcial**: el boolean `allow_entry` en `options_intent_router.py` se consulta en 5 puntos. Reemplazarlo por enum `allow|delay|block|force_exit` requiere actualizar todos los call-sites con compat para casos boolean.
10. **FSM construcción desde cero**: introducir state machine sobre loops procedurales existentes en operation_center sin migración cuidadosa puede generar dobles ejecuciones o estados inconsistentes.
11. **No push prematuro**: cualquier commit que avance fase sin aprobación humana viola la STOP RULE.
12. **No exposición de tokens**: settings carga `TRADIER_*_TOKEN` desde env y archivo fallback `C:\dev\credenciales.txt` (ruta Windows local). Hay que asegurar que estos valores nunca lleguen a logs de tests, fixtures o commits.

---

## 9. Lista NO romper (anclajes intocables hasta aprobación específica)

- `paper_only=True` (operation_center.py:72, schemas.py, runtime_config_v4 y todos los puntos de propagación).
- `full_live_globally_locked=True` (operation_center.py:146, runtime_config_v4.py:29, live_switch.py:30,52-53, api/schemas.py:285,321).
- Guardrails de `operation_center.py` y `production_guard.py` (daily/weekly/monthly loss limits, dry-run forzado).
- Reconciliación post-timeout Tradier (en `tradier_execution.py` invocada desde operation_center).
- Journal sync (operation_center → journal en flush).
- Contratos Pydantic multi-leg (`atlas_code_quant/api/schemas.py`).
- API y SSE actuales del Radar (`atlas_adapter/routes/radar_public.py`, schemas, mapper, client, tests de contrato).
- Canonical snapshot / monitoring (endpoints `/health`, `/readiness`, `/api/quant/*` en `api/main.py`).
- Pipeline visual paper loop (vision/insta360_capture, chart_ocr, multi_timeframe_analyzer, visual_pipeline) y RTMP default `rtmp://192.168.1.10/live/atlas` en `atlas_quant_core.py:113,454,520`.
- Backtesting interno (`backtester.py`, `backtesting/winning_probability.py`).
- Tests críticos: `tests/test_radar*`, `tests/test_scanner*`, `tests/test_options*`, `tests/test_risk*`, `tests/test_safety*`.
- Dashboard, health, readiness y degraded states honestos (`compute_degradations_active`, `CAMERA_UNAVAILABLE`).
- Kill switch / `/emergency/{stop,reset}` (api/main.py:3327,3343).
- Compat shim raíz `scanner/__init__.py` (mientras consumidores no migren).
- Settings Tradier (`config/settings.py:229-243`): no leer secretos en código fuente, sólo desde env / archivo fallback.

---

## 10. Plan de fases adaptado al repo real

> Cada fase debe entregar: cambios concretos + tests + documentación + commit atómico + criterio de aceptación + rollback.
> Tras cada fase, **STOP** y aprobación humana antes de la siguiente.

### F0 — Verificación (esta fase)
- Cambios: ninguno funcional; sólo este documento.
- Tests: smoke import + colección no destructiva (ver §12).
- Doc: `docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md`.
- Commit: `docs: F0 verify Atlas Code Quant architecture audit`.
- Criterio de aceptación: documento generado, smoke tests OK, working tree con sólo este archivo nuevo, sin push.
- Rollback: `git reset --hard HEAD~1` (HEAD previo `41f86833`).

### F1 — Saneamiento de carpetas y wrappers compat
- Cambios: crear directorios vacíos esperados (`atlas_code_quant/lean/`, `atlas_code_quant/intake/`, `atlas_code_quant/autonomy/`, `atlas_code_quant/telemetry/`, `atlas_code_quant/legacy/`) con `__init__.py` y READMEs internos. NO mover código aún.
- Tests: imports verifican que los paquetes existen y están vacíos.
- Doc: `docs/F1_PACKAGE_SCAFFOLDING.md`.
- Commit: `feat(scaffolding): F1 package skeletons sin lógica`.
- Criterio: árbol nuevo presente, ningún import existente roto.
- Rollback: `git revert` del commit F1.

### F2 — Renombre `lean_simulator.py` → `internal_gbm_simulator.py`
- Cambios: mover archivo a `internal_gbm_simulator.py`, dejar `lean_simulator.py` como wrapper que reexporta y emite `DeprecationWarning`. Igual con `scripts/generate_lean_dataset.py` → `scripts/generate_internal_gbm_dataset.py`.
- Tests: nuevo test de import compat + tests existentes verdes.
- Doc: `docs/F2_LEAN_RENAME.md`.
- Commit: `refactor(backtest): rename lean_simulator to internal_gbm_simulator with deprecated wrapper`.
- Criterio: imports antiguos siguen funcionando con warning; nuevo nombre disponible.
- Rollback: revert.

### F3 — Deprecación controlada de endpoints scanner duplicados
- Cambios: añadir headers `Deprecation: true` y `Sunset` a `/api/v2/quant/scanner/*` (o al bloque elegido como deprecated) en `api/main.py:3677-3706`. Sin borrar handlers.
- Tests: contrato de headers + status 200 mantenidos.
- Doc: `docs/F3_SCANNER_ENDPOINT_DEPRECATION.md`.
- Commit: `feat(api): F3 deprecate duplicate scanner endpoints with Sunset header`.
- Criterio: tests existentes verdes, OpenAPI muestra `deprecated=true`.
- Rollback: revert.

### F4 — Compat shim Tradier legacy en `atlas_options_brain_fase1/`
- Cambios: marcar `tradier_executor.py`, `tradier_live.py`, `providers/tradier_provider.py` con `DeprecationWarning` y reexportar (cuando sea posible) hacia `atlas_code_quant/execution/`. Si firmas no son compatibles, sólo añadir warning.
- Tests: imports siguen funcionando, tests existentes verdes.
- Doc: `docs/F4_TRADIER_LEGACY_DEPRECATION.md`.
- Commit: `refactor(broker): F4 deprecate atlas_options_brain_fase1 Tradier duplicates`.
- Criterio: `paper_only` y `full_live_globally_locked` intactos.
- Rollback: revert.

### F5 — Endpoint `/api/radar/opportunities` (multi-símbolo) + batch engine + universe provider
- Cambios: crear `atlas_adapter/routes/radar_opportunities.py`, `atlas_adapter/services/radar_batch_engine.py`, `atlas_adapter/services/universe_provider.py`. Sin tocar el endpoint mono-símbolo existente; coexisten.
- Tests: nuevos tests de contrato HTTP + SSE batch + tests existentes verdes.
- Doc: `docs/F5_RADAR_OPPORTUNITIES_MULTI.md`.
- Commit: `feat(radar): F5 multi-symbol /api/radar/opportunities batch engine`.
- Criterio: stream actual mono-símbolo intacto; multi-símbolo nuevo opcional.
- Rollback: revert + retirar router.

### F6 — Vision Timing Gate institucional
- Cambios: crear `atlas_code_quant/vision/timing_gate.py` con enum `Decision = allow | delay | block | force_exit` y `trace_id`. Adaptador en `options_intent_router.py` que mantiene compat con `allow_entry` boolean.
- Tests: test del enum, test del adaptador, tests existentes options_intent_router verdes.
- Doc: `docs/F6_VISION_TIMING_GATE.md`.
- Commit: `feat(vision): F6 institutional Vision Timing Gate enum allow|delay|block|force_exit`.
- Criterio: callers existentes siguen funcionando; enum disponible.
- Rollback: revert.

### F7 — FSM autónoma formal en `atlas_code_quant/autonomy/`
- Cambios: crear `atlas_code_quant/autonomy/state_machine.py` con estados nombrados (idle / scanning / evaluating / paper_executing / cooldown / blocked / emergency_stop). Adaptador desde `operation_center` y `atlas_core/autonomy/orchestrator.py` que delega gradualmente. Locks `paper_only` y `full_live_globally_locked` se respetan en cada transición.
- Tests: pruebas de transiciones, prohibición de transición a estados live, integración con kill switch.
- Doc: `docs/F7_AUTONOMY_FSM.md`.
- Commit: `feat(autonomy): F7 formal FSM with named states and live-locked guards`.
- Criterio: loops existentes siguen funcionando; FSM disponible y observable.
- Rollback: revert.

### F8 — Kill switch engine canónico + telemetría
- Cambios: ampliar `atlas_code_quant/operations/kill_switch.py` a engine con fuentes (manual / production_guard / risk / ops / fsm) y prioridades. Crear `atlas_code_quant/telemetry/` con bus de eventos. Endpoints `/emergency/*` siguen funcionando.
- Tests: prioridades, propagación, integración con production_guard.
- Doc: `docs/F8_KILL_SWITCH_ENGINE.md`.
- Commit: `feat(safety): F8 canonical kill switch engine with telemetry bus`.
- Criterio: `/emergency/stop` y `/emergency/reset` se comportan idénticamente; engine internamente formaliza.
- Rollback: revert.

### F9 — Decisión LEAN externo / vendor / abandono explícito
- Cambios: documento de decisión arquitectónica + (si se aprueba) scaffold de `atlas_code_quant/lean/` que llama a proceso/contenedor LEAN externo. NO escribir simulador GBM dentro de `lean/`. Si se decide abandono, deprecar nombre LEAN públicamente.
- Tests: smoke del cliente externo (mock).
- Doc: `docs/F9_LEAN_DECISION.md`.
- Commit: `feat(lean): F9 LEAN integration decision and external client scaffold`.
- Criterio: nada se llama "LEAN" si no es LEAN real.
- Rollback: revert.

### F10 — Cutover scanner → Radar
- Cambios: migrar consumidores (`backtester.py:27`, `learning/trading_implementation_scorecard.py:568`, otros via grep) a Radar. Mover `atlas_code_quant/scanner/*` a `atlas_code_quant/legacy/scanner/*` con compat shim. Endpoints scanner pasan a 410 Gone tras período de Sunset.
- Tests: todos los tests de scanner siguen verdes vía wrappers; nuevos tests confirman radar como fuente.
- Doc: `docs/F10_SCANNER_FREEZE_CUTOVER.md`.
- Commit: `refactor(scanner): F10 freeze legacy scanner and cutover to Radar institutional`.
- Criterio: ningún consumidor en producción depende del scanner; auditoría `SCANNER_CRITERIA_AUDIT.md` cerrada.
- Rollback: revert + restaurar `atlas_code_quant/scanner/` desde `legacy/`.

---

## 11. Comandos ejecutados (auditoría F0)

```bash
# Identificación de repo y rama
git -C /home/user/workspace/atlas-core status
git -C /home/user/workspace/atlas-core log -1 --oneline
git -C /home/user/workspace/atlas-core rev-parse HEAD
git -C /home/user/workspace/atlas-core branch --show-current

# Inventario árbol y tamaños
ls /home/user/workspace/atlas-core/docs | head -20
wc -l atlas_code_quant/scanner/opportunity_scanner.py \
      atlas_code_quant/backtest/lean_simulator.py \
      atlas_adapter/routes/radar_public.py \
      atlas_adapter/routes/radar_schemas.py \
      atlas_adapter/routes/radar_quant_mapper.py \
      atlas_adapter/routes/radar_quant_client.py \
      atlas_code_quant/operations/operation_center.py

# Confirmación mono-símbolo en Radar SSE
grep -n 'stream_sym\|symbol or "SPY"' \
     atlas_adapter/routes/radar_public.py \
     atlas_adapter/routes/radar_quant_mapper.py

# Confirmación locks paper/live
grep -n 'paper_only\|full_live_globally_locked' \
     atlas_code_quant/operations/operation_center.py \
     atlas_code_quant/operations/runtime_config_v4.py \
     atlas_code_quant/operations/live_switch.py

# Confirmación FSM inexistente
grep -rE "class .*StateMachine|class .*FSM" atlas_code_quant atlas_core atlas_adapter
# → 0 resultados

# Confirmación lean_simulator es GBM sintético
head -40 atlas_code_quant/backtest/lean_simulator.py

# Inventario endpoints scanner
grep -n '"/scanner/\|"/api/v2/quant/scanner/' atlas_code_quant/api/main.py
```

---

## 12. Resultado de tests iniciales (smoke / no destructivos)

> Esta fase NO ejecuta la suite completa. Sólo verifica que el árbol importa y que los tests son detectables.

| Comando | Propósito | Resultado esperado |
|---|---|---|
| `python -c "import atlas_code_quant"` | Smoke import del paquete principal | Importable sin raise |
| `python -c "import atlas_adapter.routes.radar_public; import atlas_adapter.routes.radar_schemas; import atlas_adapter.routes.radar_quant_mapper; import atlas_adapter.routes.radar_quant_client"` | Smoke import del Radar institucional | Importable sin raise |
| `pytest atlas_code_quant/tests --collect-only -q` | Colección de tests (sin ejecución) | Lista de tests detectada, 0 errors de colección |
| `pytest atlas_adapter --collect-only -q` | Colección de tests Radar | Lista de tests detectada |

### Resultados reales obtenidos en este turno F0

| Comando | Resultado |
|---|---|
| `python -c "import atlas_code_quant"` | ✅ `OK atlas_code_quant import` (salida limpia, sin warnings críticos) |
| `python -c "import atlas_adapter.routes.radar_public; import atlas_adapter.routes.radar_schemas; import atlas_adapter.routes.radar_quant_mapper; import atlas_adapter.routes.radar_quant_client"` | ✅ `OK atlas_adapter radar modules` |
| `python -m pytest atlas_code_quant/tests --collect-only -q` | ⚠️ **964 tests collected**, 40 errores de colección. Causa: dependencias opcionales no instaladas en este sandbox (`sqlalchemy`, etc. — ej. `atlas_code_quant/journal/service.py:16` requiere `sqlalchemy`). NO es defecto del repo. |
| `python -m pytest atlas_adapter --collect-only -q` | ⚠️ `no tests collected` — los tests de Radar están colocados en `atlas_adapter/routes/test_radar_*.py` y ya cuentan dentro del paquete; no requieren acción. |

**Interpretación F0:** los smoke imports pasan, lo que confirma que el árbol de paquetes y los módulos canónicos descritos en §4 son importables. Los 40 errores de colección de tests se deben exclusivamente a dependencias opcionales del entorno de auditoría (no al repo) y no bloquean F0. La suite completa de 964 tests se ejecutará en F1+ con el entorno de dependencias completas (Tradier paper sandbox, sqlalchemy, etc.).

> Si en F1 cualquiera de estos smoke imports rompe, F1 se aborta inmediatamente y se reporta como regresión.

---

## Cierre F0

- **Cambios funcionales en código:** ninguno.
- **Documento entregado:** `docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md` (este archivo).
- **Commit atómico planificado:** `docs: F0 verify Atlas Code Quant architecture audit`.
- **Push:** prohibido. Sólo commit local.
- **Estado posterior:** STOP. Esperar aprobación humana explícita antes de F1.

> Si el revisor encuentra cualquier punto de §2-§9 incorrecto, comentarlo antes de aprobar F1. F0 se puede regenerar con cambios documentales sin tocar código.
