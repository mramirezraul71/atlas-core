# F6 — Atlas Code Quant: Radar intake (shadow only)

Estado: F6 completada (sin push). Cambios sólo en
`atlas_code_quant/intake/`, `atlas_code_quant/config/legacy_flags.py`,
`tests/atlas_code_quant/` y `docs/`.

## 1. Motivación

F5 entregó los endpoints multi-símbolo del Radar (`/api/radar/opportunities`,
`/api/radar/opportunities/{symbol}`, `/api/radar/stream/opportunities`)
sobre `atlas_adapter`. Atlas Code Quant todavía depende del **scanner
legacy** (`atlas_code_quant/scanner/`) como única fuente efectiva de
oportunidades; F1-F4 ya marcaron ese scanner como heredado y añadieron
deprecación visible.

F6 cierra el gap del lado consumidor preparando un **cliente intake
interno** y unos **modelos internos** que mapean uno a uno el contrato
público del Radar. Es deliberadamente **shadow only**:

- Sirve para que en una fase posterior podamos comparar el output del
  scanner legacy contra el del Radar canónico (paridad scanner-vs-Radar).
- Permite probar ahora — con tests aislados — el parseo, el manejo de
  errores y la estructura interna sin tocar runtime.
- No modifica ningún loop de ejecución, ni endpoints, ni configuración
  real de paper/live.

F6 NO desengancha el scanner ni cambia el comportamiento de Code Quant en
producción. El scanner sigue siendo la única fuente efectiva.

## 2. Reglas duras respetadas (F6)

- ❌ NO ejecuta trading. ❌ NO órdenes reales. ❌ NO broker live.
- ❌ NO toca locks `paper_only`, `full_live_globally_locked`, dry-run.
- ❌ NO toca `atlas_code_quant/api/main.py`.
- ❌ NO toca scanner legacy ni sus endpoints (`/scanner/*`,
  `/api/v2/quant/scanner/*`).
- ❌ NO conecta `radar_client` a operations / execution / autonomy /
  risk / vision / live / paper loop / monitoring.
- ❌ NO toca Tradier (canónico ni phase1).
- ❌ NO modifica los endpoints Radar (`summary`, `dealer`, `diagnostics`,
  `camera`, `stream`, `opportunities*`).
- ❌ NO consume el flag `ATLAS_RADAR_INTAKE_ENABLED` en runtime (doc-only).
- ✅ Sólo añade modelos internos, cliente HTTP/SSE, flag doc-only, tests
  y este documento.
- ✅ Commit atómico local, sin push.

## 3. Archivos introducidos / modificados

| Archivo | Estado | Notas |
| --- | --- | --- |
| `atlas_code_quant/intake/opportunity.py` | **Nuevo** | Modelos internos. |
| `atlas_code_quant/intake/radar_client.py` | **Nuevo** | Cliente shadow. |
| `atlas_code_quant/config/legacy_flags.py` | Modificado | Flag doc-only `ATLAS_RADAR_INTAKE_ENABLED`. |
| `tests/atlas_code_quant/test_intake_radar_client_f6.py` | **Nuevo** | 22 tests. |
| `docs/ATLAS_CODE_QUANT_F6_RADAR_INTAKE_SHADOW.md` | **Nuevo** | Este documento. |

`atlas_code_quant/api/main.py`, scanner, operations, execution, autonomy,
Tradier, paper loop, atlas_adapter routes — **sin cambios**.

## 4. Modelos internos (`atlas_code_quant/intake/opportunity.py`)

Mapeados desde el contrato público
(`atlas_adapter.routes.radar_schemas`) sin importar Pydantic en hot-path
(usamos `dataclass(frozen=True)` para inmutabilidad y zero-cost cuando
ya tenemos un dict).

### `RadarIntakeDegradation`

Subset normalizado de `RadarDegradationEntry`. 4 campos: `code`,
`label`, `severity ∈ {info, warning, critical}`, `source`. Tolera
payloads malformados (rellena con valores honestos).

### `RadarOpportunityInternal`

Contrato esencial de una oportunidad consumida por Code Quant:

| Campo | Mapeo desde `RadarOpportunity` |
| --- | --- |
| `symbol` | `symbol` (uppercase) |
| `asset_class` | `asset_class` (lowercase) |
| `sector` | `sector` (lowercase, opcional) |
| `optionable` | `optionable` |
| `score` | `score` (float, NaN → 0.0) |
| `classification` | `classification` (validado contra `VALID_CLASSIFICATIONS`, fallback `reject`) |
| `direction` | `direction` (validado contra `VALID_DIRECTIONS`, fallback `neutral`) |
| `timestamp` | `timestamp` (string, fallback now-UTC) |
| `horizon_min` | `horizon_min` (int u opcional) |
| `snapshot` | `snapshot` (dict, sin perder claves extra) |
| `degradations_active` | `tuple[RadarIntakeDegradation, ...]` |
| `source` | `source` (validado contra `VALID_SOURCES`, fallback `stub`) |
| `trace_id` | `trace_id` |
| `raw` | dict del payload original (no participa en `__eq__`/`repr`) |

Helpers de filtrado:

- `meets_min_score(min_score)`
- `is_high_conviction`, `is_watchlist`
- `is_etf`, `is_equity`, `is_index`
- `is_live` (source == `quant`), `is_stub`
- `to_dict()` para logging / serialización.

### `RadarOpportunityBatchInternal`

Refleja `RadarOpportunitiesResponse` con un campo extra crítico:

- `degradations_active`: degradaciones del Radar (fuente upstream).
- `intake_degradations`: degradaciones **propias** que añade Code Quant
  cuando el intake en sí mismo falla (timeouts, 5xx, payload inválido).

Helpers:

- `empty_with_degradation(code, label, severity, source, trace_id)`
- `has_intake_failure`
- `filter_by_min_score(min_score)`
- `filter_by_asset_class(asset_class)`
- `to_dict()`

## 5. Cliente HTTP/SSE (`atlas_code_quant/intake/radar_client.py`)

### `RadarClientConfig`

Lee env con defaults seguros:

- `ATLAS_RADAR_BASE_URL` (default `http://localhost:8791`).
- `ATLAS_RADAR_TIMEOUT_MS` (default `5000`).

### `RadarClient`

API mínima:

```python
class RadarClient:
    def __init__(self,
                 config: RadarClientConfig | None = None,
                 *, transport: httpx.BaseTransport | None = None) -> None: ...

    def fetch_opportunities(
        self, filters: Mapping[str, Any] | None = None,
    ) -> RadarOpportunityBatchInternal: ...

    def fetch_opportunity(
        self, symbol: str,
    ) -> RadarOpportunityInternal | None: ...

    async def stream_opportunities(
        self,
        filters: Mapping[str, Any] | None = None,
        *, max_events: int | None = None,
    ) -> AsyncIterator[dict[str, Any]]: ...
```

### Función shadow a nivel módulo

```python
def fetch_opportunities_snapshot(
    filters: Mapping[str, Any] | None = None,
    *, client: RadarClient | None = None,
) -> RadarOpportunityBatchInternal: ...
```

> Pensada para una fase posterior que compare scanner vs Radar.
> **No** se llama desde `api/main.py`, operations, execution, autonomy,
> risk ni paper loop en F6.

### Filosofía de errores: "no matar el proceso"

Cualquier fallo de intake se traduce a una entrada en
`intake_degradations` y devuelve un batch vacío:

| Situación | Código de degradación interna |
| --- | --- |
| `httpx.TimeoutException` | `RADAR_TIMEOUT` |
| `httpx.ConnectError` / red caída | `RADAR_UNREACHABLE` |
| `HTTPError` genérico de transporte | `RADAR_UNREACHABLE` |
| `status >= 500` | `RADAR_HTTP_ERROR` (severity `critical`) |
| `status >= 400` (no-404) | `RADAR_HTTP_ERROR` (severity `warning`) |
| Body no decodificable | `RADAR_INVALID_PAYLOAD` |
| Body no es JSON-objeto | `RADAR_INVALID_PAYLOAD` |
| Mapper interno falla | `RADAR_INVALID_PAYLOAD` |

`fetch_opportunity(symbol)` añade un caso adicional: `404 Not Found`
devuelve `None` (no es fallo, es ausencia honesta).

Logger usado: `atlas.code_quant.intake.radar`.

## 6. Tests (`tests/atlas_code_quant/test_intake_radar_client_f6.py`)

22 tests, todos verdes localmente. Sin app FastAPI completa: usamos
`httpx.MockTransport` para inyectar respuestas determinísticas.

Cobertura:

1. Parseo de payload válido (dict) → batch interno con campos clave
   preservados (`score`, `classification`, `direction`, `horizon_min`,
   `snapshot`, `trace_id`, `degradations_active`, `source`).
2. Mapeo desde objeto Pydantic real (`RadarOpportunitiesResponse`,
   `RadarOpportunity`) preserva `trace_id` y degradaciones.
3. Item malformado dentro del batch no rompe el resto.
4. Filtros (`filter_by_min_score`, `filter_by_asset_class`).
5. `empty_with_degradation` factory.
6. `RadarClientConfig`: defaults, env override, parseo robusto de valores
   inválidos.
7. `fetch_opportunities` happy path: URL, params, parseo.
8. `fetch_opportunity` happy path.
9. `fetch_opportunities`: `RADAR_TIMEOUT`, `RADAR_UNREACHABLE`,
   `RADAR_HTTP_ERROR`, `RADAR_INVALID_PAYLOAD` (JSON no decodificable y
   payload no-objeto).
10. `fetch_opportunity`: 404 → `None`, 5xx → `None`, símbolo vacío
    cortocircuita sin tocar la red.
11. `fetch_opportunities_snapshot` con cliente inyectado.
12. Logger emite WARNING sin romper.

## 7. Flag doc-only

`atlas_code_quant/config/legacy_flags.py`:

```python
ATLAS_RADAR_INTAKE_ENABLED: bool = False
```

- Default `False`.
- **No se consulta en runtime** en F6. Ningún módulo lo lee como
  condicional.
- Documenta a auditoría / fases futuras que existe el cliente intake
  pero todavía no enruta tráfico real.
- Cambiar a `True` requiere fase posterior con tests de paridad,
  documento de cutover y aprobación humana explícita (regla dura).

## 8. Limitaciones conocidas (alcance F6)

- El cliente NO está conectado a operations, execution, autonomy, risk,
  vision, live, paper loop ni learning. Es código *durmiente*.
- El stream SSE (`stream_opportunities`) está implementado pero **no
  testeado contra un servidor real**: los tests cubren el path
  síncrono. La fase que lo conecte deberá añadir tests SSE con cancelación
  controlada.
- No se publican métricas Prometheus desde el intake (queda para fase
  posterior cuando el intake se enchufe).
- `RadarOpportunityInternal` no contiene precios, deltas, ni órdenes —
  no es un signal de ejecución, es un snapshot de oportunidad.
- El mapeo aplica fallback honesto a campos malformados; no levanta
  excepciones para preservar la regla de "no matar el proceso".

## 9. Confirmación: scanner sigue siendo única fuente

Tras F6:

- `atlas_code_quant/api/main.py`: **sin cambios**. Sigue exponiendo
  scanner legacy con su deprecación F3.
- Scanner legacy (`atlas_code_quant/scanner/`): **sin cambios**.
- Operations (`operation_center`, `production_guard`, `signal_executor`,
  `paper_loop`): **sin cambios**.
- Execution (Tradier canónico/phase1): **sin cambios**.
- Autonomy / Risk / Vision / Learning / Monitoring: **sin cambios**.

Code Quant continúa consumiendo oportunidades exclusivamente desde el
scanner legacy. F6 deja preparado — y testeado — el camino para una
futura comparación shadow scanner-vs-Radar y un eventual cutover, pero
no realiza ese cambio.

## 10. Criterio de aceptación

- ✅ `radar_client` y modelos internos existen, son testeables sin app
  completa.
- ✅ Fetch + parsing pasan tests sin levantar excepciones.
- ✅ Errores (timeout, 5xx, payload inválido, conexión, 404) generan
  estructura interna con indicadores de degradación claros.
- ✅ Scanner permanece como única fuente efectiva de oportunidades.
- ✅ Flag doc-only `ATLAS_RADAR_INTAKE_ENABLED = False` añadido.
- ✅ 22 tests verdes en
  `tests/atlas_code_quant/test_intake_radar_client_f6.py`.
- ✅ Working tree limpio antes del commit.
- ✅ Commit local atómico:
  `feat: F6 add Radar intake client and internal opportunity models (shadow only)`.
- ✅ Sin push.

## 11. Plan de rollback

Si F6 introduce regresión inesperada:

1. `git revert <commit-f6>` (el commit es atómico, sólo añade archivos
   nuevos y modifica `legacy_flags.py`).
2. Eliminar archivos nuevos manualmente como alternativa quirúrgica:
   - `atlas_code_quant/intake/opportunity.py`
   - `atlas_code_quant/intake/radar_client.py`
   - `tests/atlas_code_quant/test_intake_radar_client_f6.py`
   - `docs/ATLAS_CODE_QUANT_F6_RADAR_INTAKE_SHADOW.md`
3. Revertir el bloque añadido a `legacy_flags.py` (flag
   `ATLAS_RADAR_INTAKE_ENABLED` y entrada en `__all__`).
4. Validar `pytest tests/atlas_code_quant -q` (los nuevos tests
   desaparecen; baseline preexistente queda intacto).

Ningún cambio F6 toca runtime productivo, así que el rollback se
limita a borrar código durmiente y restaurar el archivo de flags. No
existe efecto sobre paper, live, scanner, Tradier ni Radar.

## 12. Próximas fases (referencia)

- **F7+** (no implementado en F6): conectar `RadarClient` en modo
  *paridad shadow* contra el scanner para diagnosticar discrepancias,
  con feature flag y métricas. Sigue sin enrutar tráfico real al
  Radar.
- **F8+**: cutover progresivo del scanner si paridad es estable.
- **F9-F10**: deprecación dura del scanner legacy y limpieza final.

Cada una de esas fases requiere su propio documento, tests dedicados,
commit atómico y aprobación humana explícita antes de cualquier push.
