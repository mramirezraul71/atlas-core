# F8 — Atlas Code Quant: shadow runtime consumer scanner→Radar

Estado: F8 completada (sin push). Cambios sólo en
`atlas_code_quant/monitoring/`, `atlas_code_quant/config/legacy_flags.py`,
`tests/atlas_code_quant/` y `docs/`.

## 1. Objetivo de F8

F7 dejó la API pura (filtro shadow scanner→Radar) sin enganchar a nada.
F8 introduce el primer **consumer runtime-safe** que sabe ejecutar ese
filtro contra un set real de candidatos del scanner y emitir un
**reporte agregado** apto para logging / telemetría / journal interno.

Principio rector:

> F8 observa y registra.
> F8 NO decide.
> F8 NO bloquea.
> F8 NO cambia el output efectivo del scanner.

El consumer existe, está testeado y disponible para inyección
explícita (tests, scripts internos, fases futuras). **No** introduce
hook automático en runtime; ver §3.

## 2. Reglas duras respetadas (F8)

- ❌ NO ejecuta trading. ❌ NO órdenes reales. ❌ NO broker live.
- ❌ NO toca locks `paper_only`, `full_live_globally_locked`, dry-run.
- ❌ NO toca `atlas_code_quant/api/main.py`.
- ❌ NO toca `atlas_code_quant/scanner/*`.
- ❌ NO toca `atlas_code_quant/strategies/*` (ni cambia su input).
- ❌ NO toca `atlas_code_quant/execution/*` (Tradier, broker_router,
  signal_executor, paper loop, live activation).
- ❌ NO toca `atlas_code_quant/operations/*` (operation_center,
  production_guard, sensor_vision, journal sync, kill_switch).
- ❌ NO toca autonomy FSM, risk, vision, monitoring/advanced_monitor,
  canonical_snapshot.
- ❌ NO toca `atlas_adapter` (Radar inalterado desde F5).
- ❌ NO consume las flags `ATLAS_RADAR_FILTER_SHADOW_ENABLED` ni
  `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED` en runtime — la prueba
  estática `test_no_runtime_module_consults_runtime_flag` lo verifica.
- ✅ Sólo añade un módulo nuevo en `monitoring/`, una flag doc-only,
  tests y este documento.
- ✅ Commit atómico local, sin push.

## 3. Decisión consciente: F8 NO introduce hook en runtime

F8 mantiene la API disponible pero NO la engancha a ningún loop. Las
razones explícitas:

- Cualquier hook real, aun en monitoring puro, requiere un punto que
  reciba el output del scanner ya materializado. Los puntos candidatos
  (`monitoring/strategy_tracker.py`, `monitoring/advanced_monitor.py`,
  `operations/sensor_vision.py`) tienen acoplamiento con el live loop
  o con el journal sync, y meter ahí una llamada a Radar — aun en
  modo log-only — implica:
    - latencia adicional síncrona en hot-path, o
    - una pieza de orquestación (cola asíncrona, thread pool) que
      F8 no necesita testear todavía,
    - revisión de invariantes de timing que esos módulos asumen.
- El alcance F8 indica: *si no existe un punto claramente seguro, no
  fuerces el hook; deja la función preparada y bien documentada*.
- F9 puede introducir el hook con su propio plan de paridad y
  rollback.

Lo que F8 sí entrega:

- `run_scanner_radar_shadow(...)` **inyectable** desde tests / scripts /
  futuras fases, con la misma garantía de no-side-effect operativo.
- Una flag `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED` lista para que
  F9 la consulte cuando introduzca el hook.

## 4. Archivos introducidos / modificados

| Archivo | Estado | Notas |
| --- | --- | --- |
| `atlas_code_quant/monitoring/scanner_radar_shadow.py` | **Nuevo** | Consumer runtime-safe + reporte. |
| `atlas_code_quant/config/legacy_flags.py` | Modificado | Flag `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`. |
| `tests/atlas_code_quant/test_scanner_radar_shadow_f8.py` | **Nuevo** | 16 tests. |
| `docs/ATLAS_CODE_QUANT_F8_SCANNER_RADAR_RUNTIME_SHADOW.md` | **Nuevo** | Este documento. |

`api/main.py`, scanner, strategies, operations, execution, autonomy,
atlas_adapter, locks, production_guard, kill_switch, vision —
**sin cambios**.

## 5. API entregada

### `ScannerRadarShadowReport`

```python
@dataclass(frozen=True)
class ScannerRadarShadowReport:
    scanner_candidate_count: int
    radar_approved_count: int
    radar_rejected_count: int
    approved_symbols: tuple[str, ...]
    rejected_symbols: tuple[str, ...]
    degradations: tuple[RadarIntakeDegradation, ...]
    metadata: dict[str, Any]
    filter_result: ScannerRadarFilterResult  # repr=False, compare=False

    @property
    def divergence_ratio(self) -> float: ...
    @property
    def has_batch_degradation(self) -> bool: ...
    def to_dict(self) -> dict[str, Any]: ...
```

Notas:

- `scanner_candidate_count` refleja la entrada original (incluso si el
  Radar está caído) → permite calcular cobertura y divergencia con
  honestidad.
- `divergence_ratio = radar_rejected_count / scanner_candidate_count`
  (0.0 si la entrada es vacía). Métrica clave para vigilar la tasa de
  discrepancia scanner-vs-Radar.
- `metadata` heredado del `ScannerRadarFilterResult` de F7
  (`trace_id`, `timestamp`, `min_score`, `radar_*`, `batch_failed`).
- `filter_result` se preserva entero por si un consumer (futuro)
  quiere bajar al detalle por símbolo.

### Función pública

```python
def run_scanner_radar_shadow(
    scanner_candidates: Sequence[Any] | Iterable[Any],
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,
    emit_logs: bool = True,
) -> ScannerRadarShadowReport: ...
```

Comportamiento:

1. Materializa la entrada (acepta iterables, ``None``, listas vacías).
2. Emite `scanner_radar_shadow_started`.
3. Si la entrada es vacía → reporte vacío y `scanner_radar_shadow_completed`.
4. Llama `filter_scanner_candidates_with_radar` (F7).
5. Si el filtro lanzase excepción inesperada (defense in depth): se
   construye un reporte degradado con código `RADAR_UNREACHABLE`.
6. Si hay degradaciones de batch → emite
   `scanner_radar_shadow_degraded`.
7. Siempre emite `scanner_radar_shadow_completed`.

NUNCA lanza al caller.

## 6. Logging estructurado

Logger: `atlas.code_quant.shadow.scanner_radar`.

Eventos:

| Evento | Nivel | Payload extra |
| --- | --- | --- |
| `scanner_radar_shadow_started` | INFO | `scanner_count`, `min_score` |
| `scanner_radar_shadow_completed` | INFO | `scanner_count`, `approved_count`, `rejected_count`, `approved_symbols` (truncado), `rejected_symbols` (truncado), `divergence_ratio`, `min_score`, `trace_id` |
| `scanner_radar_shadow_degraded` | WARNING | `scanner_count`, `degradations`, `trace_id` |

Truncado: si la lista de símbolos supera `MAX_LOGGED_SYMBOLS = 25`, el
log incluye los 25 primeros y un marcador `…(+N)` con los restantes.
Esto evita spam cuando el scanner produce listas grandes.

`emit_logs=False` desactiva todos los eventos del logger F8 (útil para
tests / scripts batch que no quieren ruido).

## 7. Ejemplos de divergencia (desde tests)

| Caso | Scanner input | Radar approved | Radar rejected | divergence_ratio | degradations |
| --- | --- | --- | --- | --- | --- |
| Path feliz (AAPL high, SPY watchlist, MSFT score 40) | 3 | 2 (AAPL, SPY) | 1 (MSFT) | 0.333 | () |
| Radar timeout | 2 | 0 | 2 | 1.000 | RADAR_TIMEOUT |
| Radar 503 | 1 | 0 | 1 | 1.000 | RADAR_HTTP_ERROR |
| Radar JSON inválido | 1 | 0 | 1 | 1.000 | RADAR_INVALID_PAYLOAD |
| Radar connect error | 2 | 0 | 2 | 1.000 | RADAR_UNREACHABLE |
| Filtro F7 lanza excepción (defense in depth) | 1 | 0 | 0 | 0.000 | RADAR_UNREACHABLE (origen: shadow) |
| Lista vacía | 0 | 0 | 0 | 0.000 | () |

## 8. Confirmaciones explícitas

- ✅ Scanner sigue siendo única fuente efectiva hacia estrategias. F8
  NO altera ningún wiring de `atlas_code_quant/strategies/*` ni
  `atlas_code_quant/scanner/*`.
- ✅ F8 NO modifica endpoints ni rutas públicas. `api/main.py`
  intacto.
- ✅ F8 NO altera ejecución de órdenes. `execution/*` y
  `operations/*` sin cambios.
- ✅ F8 NO toca el Radar (`atlas_adapter`).
- ✅ La flag `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED` permanece
  `False` y NO se consulta en runtime — verificado por test estático.
- ✅ El consumer no está importado desde scanner / strategies /
  execution / api — verificado por test estático que recorre
  textualmente esos paquetes.

## 9. Tests

`tests/atlas_code_quant/test_scanner_radar_shadow_f8.py` — **16 tests
verdes** con `httpx.MockTransport` y `unittest.mock.patch`. Cobertura:

1. Path feliz: counts, símbolos, divergence_ratio, trace_id, no
   degradación.
2. Lista vacía: reporte vacío sin tocar Radar.
3. `None` en entrada: tratado como vacío.
4. Fallos transport (`ReadTimeout`, `ConnectError`) → reporte degradado
   con `scanner_count` preservado.
5. HTTP 5xx → `RADAR_HTTP_ERROR`.
6. JSON inválido → `RADAR_INVALID_PAYLOAD`.
7. `emit_logs=True` produce `started` + `completed` en el logger F8.
8. `emit_logs=True` con fallo añade `degraded`.
9. `emit_logs=False` silencia todo el logger F8.
10. Truncado de listas grandes (>25 símbolos) en logs.
11. Defense in depth: filtro F7 lanza excepción → reporte degradado.
12. El consumer NO muta la entrada del scanner.
13. Flag `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED` default `False`.
14. El módulo F8 NO referencia la flag runtime (verificación textual).
15-16. Ningún módulo bajo `scanner/`, `strategies/`, `execution/`,
    `api/` importa `scanner_radar_shadow` ni llama
    `run_scanner_radar_shadow`.

Baseline `pytest atlas_code_quant/tests --collect-only`:
**1054 tests, 40 errors preexistentes** — sin regresión.

## 10. Diagrama del flujo (post-F8)

```
   ┌────────────────┐
   │   Scanner      │  (legacy, intacto)
   │   produce      │
   │   candidatos   │
   └─────┬──────────┘
         │
         ▼
   ┌────────────────┐                         ┌──────────────┐
   │  Estrategias   │  ◄── única ruta efectiva │  Execution   │
   │  (intactas)    │  ─────────────────────►  │  (intacta)   │
   └────────────────┘                          └──────────────┘

         (NO conectado en F8 — disponible para inyección)
   ┌──────────────────────────────────────┐
   │  monitoring.scanner_radar_shadow     │
   │  run_scanner_radar_shadow(...)       │
   │      ⇣ (reutiliza F7)                │
   │  intake.scanner_radar_filter         │
   │  filter_scanner_candidates_...       │
   │      ⇣ (reutiliza F6)                │
   │  intake.radar_client RadarClient     │
   └─────┬────────────────────────────────┘
         │
         ▼ (httpx)
   ┌────────────────┐
   │ atlas_adapter  │  /api/radar/opportunities
   │   Radar (F5)   │
   └────────────────┘

   logger: atlas.code_quant.shadow.scanner_radar
       events: started / completed / degraded
```

## 11. Plan de rollback

Commit atómico local: `feat: F8 add runtime-safe scanner→Radar shadow consumer`.

Para revertir:

1. `git revert <commit-f8>` (commit atómico, sólo añade archivos
   nuevos y modifica `legacy_flags.py`).
2. Alternativa quirúrgica:
   - eliminar `atlas_code_quant/monitoring/scanner_radar_shadow.py`,
   - eliminar `tests/atlas_code_quant/test_scanner_radar_shadow_f8.py`,
   - eliminar `docs/ATLAS_CODE_QUANT_F8_SCANNER_RADAR_RUNTIME_SHADOW.md`,
   - revertir el bloque del flag `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`
     en `legacy_flags.py` (incluyendo la entrada en `__all__`).
3. Validar `pytest tests/atlas_code_quant -q` — los 16 tests F8
   desaparecen; F6 (22) y F7 (15) siguen verdes.

Ningún cambio F8 toca runtime productivo, así que el rollback se
limita a borrar código durmiente y restaurar el archivo de flags. No
hay efecto sobre paper, live, scanner, Tradier, Radar, estrategias ni
endpoints.

## 12. Próximas fases (referencia)

- **F9**: introducir el *hook* de observación detrás de
  `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`. Localización candidata:
  un sensor dedicado en `monitoring/` que se suscriba al output del
  scanner sin entrar en su hot-path (cola asíncrona / event bus
  interno). Tests de paridad scanner-vs-Radar y métricas Prometheus.
- **F10**: `ATLAS_RADAR_FILTER_ENFORCED` para gate duro por
  estrategia, con rollback rápido y aprobación humana explícita.
- **F11+**: deprecación final del scanner como fuente directa hacia
  estrategias.

Cada una de esas fases requiere su propio documento, tests dedicados,
commit atómico y aprobación humana explícita antes de cualquier push.
