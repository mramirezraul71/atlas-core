# Atlas Code Quant — F9: Runtime hook scanner→Radar shadow + preparación de enforcement

> **Estado:** F9 implementada y aprobada (paper-only, observacional).
> **Defaults:** `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED = False`,
> `ATLAS_RADAR_FILTER_ENFORCED = False`. Ningún loop de producción
> consume todavía el hook. F10+ planificará el enganche real y el
> gate duro.

---

## 1. Objetivo

F9 cierra la brecha entre los componentes shadow (F6 intake, F7 filter,
F8 consumer) y la pipeline real, **sin tocar la pipeline real**. Concretamente:

1. Identifica y documenta el **punto de corte** scanner → consumidores
   en el repo (sólo lectura).
2. Introduce un **hook opt-in** runtime-safe
   (`maybe_run_scanner_radar_shadow`) que sólo se activa si
   `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED == True`. Default `False`.
3. Declara la flag **doc-only** `ATLAS_RADAR_FILTER_ENFORCED` para el
   gate duro futuro. F9 NO la consulta en runtime.

**No alcance F9:**

* No se modifica `api/main.py`.
* No se modifica `scanner/`, `strategies/`, `execution/`.
* No se modifica `atlas_adapter/`, autonomy FSM, risk, vision, locks,
  `production_guard`.
* No se hace push.

---

## 2. Punto de corte scanner → consumidores (lectura, NO modificación)

### 2.1 Búsqueda

```bash
grep -rn "OpportunityScannerService\|from atlas_code_quant.scanner" \
    atlas_code_quant/ --include="*.py" \
  | grep -v "atlas_code_quant/scanner/" \
  | grep -v "atlas_code_quant/tests/"
```

### 2.2 Resultado

El **único consumidor activo del scanner en producción** vive en
`atlas_code_quant/api/main.py`:

| Línea (api/main.py) | Llamada | Salida del scanner |
|----|----|----|
| `119` | `from scanner.opportunity_scanner import OpportunityScannerService` | (import) |
| `218` | `_SCANNER = OpportunityScannerService(_ADAPTIVE_LEARNING)` | instancia singleton |
| `458 / 560 / 630` | `await _SCANNER.start()` | arranca el motor (no produce candidatos al caller) |
| `532 / 600` | `configure_operational_briefing(scanner=_SCANNER, ...)` | inyecta el scanner al briefing operativo |
| `1700` | `_compact_scanner_report(_SCANNER.report(24))` | compacta el reporte para SSE |
| `2418` | `report = await asyncio.to_thread(_SCANNER.report, 24)` | reporte para feed lightweight |
| `3458` | `payload = ScannerReportPayload.model_validate(_SCANNER.report(activity_limit=...))` | endpoint legacy `/scanner/report` |
| `3492` | `payload = ScannerReportPayload.model_validate(_SCANNER.report())` | tras `update_config` (legacy) |
| `3521` | `payload = ScannerReportPayload.model_validate(await _SCANNER.control(action))` | endpoint legacy `/scanner/control` |
| `720`  | `await _SCANNER.stop()` | apagado |

Otros consumidores no productivos:

* `atlas_code_quant/backtester.py:27` — usa `OpportunityScannerService` como
  fuente histórica para backtests internos. **No es ruta de live.**
* `atlas_code_quant/notifications/payloads.py:32` — llama
  `scanner.report(activity_limit=...)` para los briefings.

**Conclusión:** los **candidatos** del scanner se exponen únicamente vía
`OpportunityScannerService.report(...)` (firma:
`scanner/opportunity_scanner.py:960`). El "punto de corte" lógico hacia
estrategias / ejecución pasa por:

1. La instancia singleton `_SCANNER` en `api/main.py:218`.
2. `briefing_service.configure_operational_briefing(scanner=_SCANNER, ...)`
   en `api/main.py:532` y `:600` — *único* consumidor que recibe la
   instancia entera y puede observar candidatos en cada ciclo del
   briefing.
3. Los endpoints HTTP legacy `/scanner/report` y `/scanner/control`,
   ya marcados como `deprecated=True` en F3.

**No existe** consumidor del scanner bajo `operations/`, `strategies/`
ni `monitoring/` que pueda servir como punto de enganche del hook
**sin modificar** módulos prohibidos por el alcance F9.

> **Implicación para F9:** el hook se queda *ready to call* en
> `monitoring/scanner_radar_shadow_hook.py`, pero NO se conecta a
> ningún loop real. Su activación efectiva (probable F10) requerirá:
>
> * añadir un wrapper en `notifications/briefing_service.py` o un
>   adaptador en `operations/` que tome el output de `_SCANNER.report()`
>   y lo pase al hook;
> * o introducir un *observer* explícito en el ciclo del scanner,
>   con su propia fase y su plan de cutover.

---

## 3. Diseño del hook

### 3.1 Ubicación

```
atlas_code_quant/monitoring/scanner_radar_shadow_hook.py
```

Se aísla en un módulo nuevo a propósito para preservar la invariante F8:

> `tests/atlas_code_quant/test_scanner_radar_shadow_f8.py::`
> `test_no_runtime_module_consults_runtime_flag`
>
> Lee el cuerpo de `scanner_radar_shadow.py` y exige que el string
> `"ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED"` **no aparezca** en él.

Si F9 hubiera añadido el `if` directamente en `scanner_radar_shadow.py`,
esa invariante caería. Separar el hook mantiene F8 inmutable y deja el
único punto runtime que consulta la flag dentro de un archivo nuevo y
auditable.

Adicionalmente, el módulo se ubica bajo `monitoring/` (no bajo
`scanner/`, `strategies/`, `execution/` ni `api/`) para no romper la
otra invariante F8:

> `test_no_module_under_scanner_or_strategies_imports_shadow_consumer`

### 3.2 API

```python
from typing import Any, Iterable, Optional, Sequence

from atlas_code_quant.intake.radar_client import RadarClient
from atlas_code_quant.intake.scanner_radar_filter import DEFAULT_MIN_SCORE
from atlas_code_quant.monitoring.scanner_radar_shadow import (
    ScannerRadarShadowReport,
)


def maybe_run_scanner_radar_shadow(
    scanner_candidates: Sequence[Any] | Iterable[Any] | None,
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,
    emit_logs: bool = True,
) -> Optional[ScannerRadarShadowReport]:
    ...
```

### 3.3 Comportamiento

* Si `legacy_flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED is False`
  (default): devuelve `None` **sin**:
  - construir `RadarClient`,
  - iterar `scanner_candidates`,
  - emitir logs del logger F8 `atlas.code_quant.shadow.scanner_radar`.
* Si la flag es `True`: delega 1:1 a `run_scanner_radar_shadow` (F8) y
  devuelve el `ScannerRadarShadowReport` resultante.
* Lectura **dinámica** de la flag (`getattr(legacy_flags, ..., False)`):
  permite a tests usar `monkeypatch.setattr` sin reimports.
* NUNCA muta `scanner_candidates`.
* NUNCA lanza al caller (heredado de F8).

### 3.4 Logger

`atlas.code_quant.shadow.scanner_radar.hook` (independiente del logger
F8 para que el toggle de la flag sea observable sin pisar telemetría).
F9 no emite eventos por sí mismo: el logging útil sigue viniendo de
F8 cuando la flag está activa.

---

## 4. Flag `ATLAS_RADAR_FILTER_ENFORCED` (doc-only)

* **Archivo:** `atlas_code_quant/config/legacy_flags.py`.
* **Default:** `False`.
* **Estado en F9:** doc-only. Ningún módulo runtime la consulta.
* **Roadmap (F10+):** cuando esté `True`, Radar se vuelve gate duro:
  los candidatos del scanner que no superen el filtro Radar (F7) no
  alimentan a estrategias.
* **Pre-requisitos para activarla:**
  - paridad shadow-vs-real verde (métricas estables de
    `divergence_ratio` durante un periodo significativo);
  - documento de cutover dedicado (F10+);
  - kill-switch coherente con `paper_only` y
    `full_live_globally_locked`;
  - tests de no-regresión y rollback explícito.

Test estático en F9 verifica que la flag NO se consulta:

```python
test_enforcement_flag_not_consulted_in_runtime_modules
```

---

## 5. Tests F9

Archivo: `tests/atlas_code_quant/test_scanner_radar_shadow_hook_f9.py`.

Cobertura:

| Categoría | Tests |
|----|----|
| Default off | `test_runtime_flag_default_remains_false`, `test_hook_returns_none_when_runtime_flag_disabled`, `test_hook_disabled_does_not_consume_iterable`, `test_hook_disabled_emits_no_f8_logs` |
| Delegación con flag on | `test_hook_enabled_delegates_to_run_scanner_radar_shadow`, `test_hook_enabled_forwards_kwargs`, `test_hook_enabled_with_empty_input_returns_empty_report`, `test_hook_enabled_with_none_returns_empty_report` |
| Degradaciones (no raise) | `test_hook_handles_transport_failures` (timeout/connect), `test_hook_handles_http_5xx`, `test_hook_handles_invalid_payload` |
| No-side-effect | `test_hook_does_not_mutate_scanner_input` |
| Enforcement gate doc-only | `test_enforcement_flag_exists_and_default_false`, `test_enforcement_flag_not_consulted_in_runtime_modules` |
| Static guards | `test_f8_invariant_preserved_runtime_flag_absent_from_f8_module`, `test_hook_module_consults_runtime_flag_textually`, `test_no_module_under_scanner_strategies_execution_api_imports_hook`, `test_hook_module_logger_name_canonical` |

Todos verdes localmente (19 tests F9 + 53 F6/F7/F8 = **72** tests
shadow verdes).

Baseline `python -m pytest atlas_code_quant/tests --collect-only -q`:
**1054 tests / 40 errores preexistentes** — no regresión.

---

## 6. Ejemplos de uso (para fases futuras)

### 6.1 Activación temporal en un script de auditoría

```python
import atlas_code_quant.config.legacy_flags as flags
from atlas_code_quant.monitoring.scanner_radar_shadow_hook import (
    maybe_run_scanner_radar_shadow,
)

flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED = True
candidates = scanner.report(activity_limit=24).get("candidates") or []
report = maybe_run_scanner_radar_shadow(candidates, emit_logs=True)
if report:
    print(report.to_dict())
```

### 6.2 Punto de enganche futuro (NO incluido en F9)

```python
# Pseudocódigo — F10+
def _on_scanner_cycle(scanner_report: dict) -> None:
    candidates = scanner_report.get("candidates") or []
    shadow = maybe_run_scanner_radar_shadow(candidates, emit_logs=True)
    if shadow is not None and shadow.has_batch_degradation:
        # observar/alertar, pero no bloquear
        ...
```

---

## 7. Reglas duras respetadas

1. ✅ Sin live trading; sin órdenes reales.
2. ✅ No se conecta broker live.
3. ✅ Sin secretos en commits.
4. ✅ Sin push.
5. ✅ Sin borrar código funcional.
6. ✅ `paper_only`, `full_live_globally_locked` intactos; defaults
   doc-only.
7. ✅ No se finge LEAN real ni se renombra el simulador GBM interno.
8. ✅ Sin movimientos masivos.
9. ✅ Sin cambios de semántica de endpoints.
10. ✅ Sin tocar lógica live.
11. ✅ Stubs claramente etiquetados (todo F9 lleva docstring "shadow"
    y la flag enforcement viene rotulada como "doc-only").
12. ✅ Cambios + tests + docs + commit atómico + criterio aceptación
    + rollback.

---

## 8. Criterio de aceptación

* [x] Hook `maybe_run_scanner_radar_shadow` existe y se queda no-op
  con flag `False`.
* [x] Hook delega a F8 con flag `True` y nunca lanza.
* [x] Hook NO está bajo `scanner/`, `strategies/`, `execution/` ni
  `api/`.
* [x] Hook NO es importado desde `scanner/`, `strategies/`,
  `execution/` ni `api/` (test estático).
* [x] Flag `ATLAS_RADAR_FILTER_ENFORCED = False` documentada y
  exportada en `__all__`.
* [x] Flag enforcement NO se consulta en runtime (test estático).
* [x] Invariante F8 sigue verde (la flag runtime NO aparece textual
  en `scanner_radar_shadow.py`).
* [x] Tests F6/F7/F8/F9 verdes (72/72).
* [x] Baseline `1054/40` preservado.

---

## 9. Rollback

* Revertir el commit F9
  (`feat: F9 add runtime hook for scanner→Radar shadow and prepare enforcement gate`)
  con `git revert <sha>`.
* Estado funcional resultante: idéntico a F8. Los módulos shadow
  F6/F7/F8 siguen disponibles y testeados.
* No hay efectos colaterales sobre `api/main.py`, scanner, execution,
  autonomy ni atlas_adapter — ninguno consumió código F9.

---

## 10. Próximos pasos (F10 propuesto, fuera de F9)

1. Identificar el adaptador concreto donde inyectar el hook (probable
   `notifications/briefing_service.py`, con su propia fase y test
   suite).
2. Ejecutar paridad shadow-vs-real durante un período suficiente
   con `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED=True` y métricas de
   `divergence_ratio` recolectadas.
3. Sólo después: plan de cutover para flippear
   `ATLAS_RADAR_FILTER_ENFORCED` con kill-switch, paridad verde y
   documento dedicado.
