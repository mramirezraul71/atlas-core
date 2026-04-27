# F7 — Atlas Code Quant: filtro shadow scanner→Radar

Estado: F7 completada (sin push). Cambios sólo en
`atlas_code_quant/intake/`, `atlas_code_quant/config/legacy_flags.py`,
`tests/atlas_code_quant/` y `docs/`.

## 1. Motivación

El objetivo de negocio del refactor es desplazar la decisión
"esto es una oportunidad" desde el scanner al Radar:

- **scanner = generador amplio** (recall): produce candidatos del
  universo curado con su lógica histórica (heurísticas, ML ranker,
  etc.).
- **Radar = filtro estricto** (precision): consume candidatos, calcula
  un score 0–100 compuesto, clasifica `high_conviction` / `watchlist`
  / `reject` y publica `degradations_active` por símbolo.

El estado destino es: ningún candidato llega a estrategias sin pasar
por la aprobación del Radar. F7 NO realiza ese cutover. F7 sólo:

1. Implementa la **API interna pura** que sabe consultar Radar y
   clasificar el set actual del scanner contra esa política, y
2. Deja la pieza testeada y observable, lista para que la fase
   siguiente (F8) la conecte en modo *paridad shadow* (telemetría /
   journal interno) y, finalmente, en modo *enforced*.

## 2. Reglas duras respetadas (F7)

- ❌ NO ejecuta trading. ❌ NO órdenes reales. ❌ NO broker live.
- ❌ NO toca locks `paper_only`, `full_live_globally_locked`, dry-run.
- ❌ NO toca `atlas_code_quant/api/main.py`.
- ❌ NO toca scanner legacy ni sus endpoints.
- ❌ NO toca `atlas_code_quant/execution/*` (Tradier, broker_router,
  signal_executor, paper loop, live activation).
- ❌ NO toca `atlas_code_quant/operations/*` (operation_center,
  production_guard, sensor_vision, journal sync).
- ❌ NO toca autonomy FSM, risk, vision, monitoring.
- ❌ NO toca `atlas_adapter` (Radar permanece tal cual quedó en F5).
- ❌ NO consume el flag `ATLAS_RADAR_FILTER_SHADOW_ENABLED` en runtime
  (doc-only).
- ❌ NO introduce hooks en runtime loops. La función nueva existe pero
  **no se llama desde ningún módulo de producción**.
- ✅ Sólo añade un módulo intake nuevo, flag doc-only, tests y este
  documento.
- ✅ Commit atómico local, sin push.

### Decisión consciente: sin hook en operations en F7

El alcance permitía considerar un hook en `operations/*` para alimentar
el filtro con candidatos reales. **No lo introduzco en F7**:

- El alcance preferente del prompt es `intake/strategies` primero.
- Cualquier hook en operations toca el live loop, aunque sea por modo
  paper. F8 puede introducir un *consumer* shadow dedicado en
  `operations/sensor_*` o `monitoring/` con su propio plan de tests
  paritarios y aprobación.
- En F7 la API queda disponible para que los tests, scripts internos
  futuros y la fase F8 la inyecten explícitamente cuando exista
  comparativa scanner-vs-Radar de verdad.

## 3. Archivos introducidos / modificados

| Archivo | Estado | Notas |
| --- | --- | --- |
| `atlas_code_quant/intake/scanner_radar_filter.py` | **Nuevo** | Servicio shadow + estructuras. |
| `atlas_code_quant/config/legacy_flags.py` | Modificado | Flag doc-only `ATLAS_RADAR_FILTER_SHADOW_ENABLED`. |
| `tests/atlas_code_quant/test_scanner_radar_filter_f7.py` | **Nuevo** | 15 tests. |
| `docs/ATLAS_CODE_QUANT_F7_SCANNER_RADAR_FILTER_SHADOW.md` | **Nuevo** | Este documento. |

`api/main.py`, scanner, operations, execution, autonomy, atlas_adapter,
estrategias — **sin cambios**.

## 4. API interna

### Estructuras (`atlas_code_quant/intake/scanner_radar_filter.py`)

#### `ScannerCandidateLike`

Vista mínima estable de un candidato del scanner:

```python
@dataclass(frozen=True)
class ScannerCandidateLike:
    symbol: str
    asset_class: str = "unknown"
    score: float = 0.0
    raw: dict[str, Any] = field(default_factory=dict, ...)

    @classmethod
    def from_any(cls, obj: Any) -> "ScannerCandidateLike": ...
```

`from_any` acepta:

- `dict[str, Any]` (formato actual del `OpportunityScannerService`),
- `dataclass`-like / cualquier objeto con atributos `.symbol`,
  `.asset_class`, `.score`,
- `ScannerCandidateLike` (idempotente).

Símbolos vacíos se preservan como `""` y luego se descartan en la
normalización.

#### `RadarFilteredCandidate`

```python
@dataclass(frozen=True)
class RadarFilteredCandidate:
    symbol: str
    scanner_candidate: ScannerCandidateLike
    radar_opportunity: RadarOpportunityInternal | None
    classification: str  # "high_conviction" | "watchlist" | "reject" | "no_data"
    score: float
    degradations: tuple[RadarIntakeDegradation, ...]
```

- `is_approved`: `classification ∈ {high_conviction, watchlist}`.
- `is_rejected`: `classification ∈ {reject, no_data}`.

#### `ScannerRadarFilterResult`

```python
@dataclass(frozen=True)
class ScannerRadarFilterResult:
    approved_by_radar: tuple[RadarFilteredCandidate, ...]
    rejected_by_radar: tuple[RadarFilteredCandidate, ...]
    radar_degradations: tuple[RadarIntakeDegradation, ...]
    metadata: dict[str, Any]
```

Metadata canónica:

- `trace_id`: trace_id del batch del Radar.
- `timestamp`: ISO-8601 UTC.
- `min_score`: umbral aplicado.
- `scanner_input`: nº de candidatos normalizados.
- `radar_returned`, `radar_universe_size`, `radar_source`.
- `approved_count`, `rejected_count`.
- `batch_failed`: `True` si hubo degradación de batch.

### Función pública

```python
def filter_scanner_candidates_with_radar(
    scanner_candidates: Sequence[Any] | Iterable[Any],
    radar_client: RadarClient | None = None,
    *,
    min_score: float = 70.0,
) -> ScannerRadarFilterResult: ...
```

Comportamiento:

1. Normaliza candidatos vía `ScannerCandidateLike.from_any`. Descarta
   símbolos vacíos / duplicados.
2. Si lista vacía → resultado vacío sin abrir socket.
3. Llama `radar_client.fetch_opportunities({"symbols": [...]})`
   (usa `RadarClient` por defecto si el caller no pasa uno).
4. Indexa el batch del Radar por símbolo.
5. Aplica política F7 (ver §5) a cada candidato del scanner.
6. Acumula approved / rejected / degradaciones de batch / metadata.

### Política de clasificación F7

| Caso | Resultado |
| --- | --- |
| Radar tiene oportunidad para el símbolo y `score ≥ min_score` y `classification ∈ {high_conviction, watchlist}` | classification nativa, en `approved_by_radar` |
| Radar tiene oportunidad pero `score < min_score` | `reject`, en `rejected_by_radar` |
| Radar tiene oportunidad con `classification == "reject"` (cualquier score) | `reject`, en `rejected_by_radar` |
| Radar NO tiene oportunidad para el símbolo | `no_data` con `RADAR_NO_DATA` info-degradation per-symbol |
| Fallo de batch (timeout / 5xx / unreachable / payload inválido) | TODOS los símbolos → `no_data`; degradación del fetch propagada a `radar_degradations` |
| Cualquier excepción no controlada en el cliente | Se traduce a `RADAR_UNREACHABLE` y se sigue (no levanta) |

`min_score=70.0` es el default conservador: la política Radar de F5 ya
clasifica `high_conviction` ≥ 80 y `watchlist` ≥ 50; el filtro F7 corta
en el medio para alinearse con la intención "Radar como filtro
estricto". Estrategias futuras pueden ajustar este umbral.

## 5. Escenarios de degradación (cubiertos por tests)

- **Radar caído / timeout**: `httpx.ReadTimeout` →
  `RADAR_TIMEOUT`. Todos los candidatos quedan en `rejected_by_radar`
  con `classification == "no_data"`. **El sistema NO entra en pánico**:
  como F7 no está conectado a runtime, esto se manifiesta solo en el
  resultado devuelto al caller (futuros consumidores en F8).
- **Radar inalcanzable**: `httpx.ConnectError` → `RADAR_UNREACHABLE`,
  comportamiento idéntico a timeout.
- **Radar HTTP 5xx**: `RADAR_HTTP_ERROR` con severity `critical`.
- **Radar JSON malformado**: `RADAR_INVALID_PAYLOAD`.
- **Radar sin datos para algunos símbolos**: classification `no_data`
  por símbolo con `RADAR_NO_DATA` info-degradation; el resto del batch
  se procesa normalmente.
- **Radar score bajo**: `reject` explícito; el scanner sigue teniendo
  el candidato en su set original (que F7 NO modifica).

## 6. Confirmaciones explícitas

- ✅ Scanner sigue siendo la única fuente efectiva de candidatos para
  estrategias. F7 NO altera ningún wiring de
  `atlas_code_quant/strategies/*`.
- ✅ F7 NO modifica endpoints ni rutas públicas. `api/main.py` queda
  intacto.
- ✅ F7 NO altera la ejecución de órdenes. `execution/*` y
  `operations/*` no cambian.
- ✅ F7 NO toca el Radar (`atlas_adapter`). Se limita a consumirlo via
  el `RadarClient` de F6.
- ✅ El flag `ATLAS_RADAR_FILTER_SHADOW_ENABLED` permanece en `False` y
  no se consulta en runtime; cualquier activación de gate duro
  requerirá una flag distinta (`ATLAS_RADAR_FILTER_ENFORCED`) y un
  documento de cutover dedicado en una fase posterior.

## 7. Tests

`tests/atlas_code_quant/test_scanner_radar_filter_f7.py` — 15 tests
verdes con `httpx.MockTransport`. Cobertura:

1. `ScannerCandidateLike.from_any` con dict, dataclass-like, vacíos.
2. Path feliz con 3 símbolos: classification correcta, score
   preservado, `params["symbols"]` enviado al Radar.
3. `min_score` configurable (80 corta SPY watchlist).
4. Ausencia de datos por símbolo → `no_data` con `RADAR_NO_DATA`
   per-symbol.
5. Fallos transport: `RADAR_TIMEOUT`, `RADAR_UNREACHABLE`,
   `RADAR_HTTP_ERROR`, `RADAR_INVALID_PAYLOAD` — todos terminan en
   `radar_degradations` y rejected.
6. Lista vacía no abre socket.
7. Símbolos duplicados / vacíos / `None` se descartan en
   normalización.
8. Excepción no controlada del cliente → `RADAR_UNREACHABLE` (defensa
   última).
9. `classification == "reject"` nativo del Radar pasa a rejected
   incluso con score alto (defensivo).
10. El filtro NO muta la lista de entrada del scanner ni sus dicts.

Baseline `pytest atlas_code_quant/tests --collect-only -q`:
**1054 tests, 40 errors preexistentes** — sin regresión.

## 8. Diagrama del flujo (post-F7)

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

         (NO conectado en F7)
   ┌────────────────────────────────┐
   │  intake.scanner_radar_filter   │  ⇢ ScannerRadarFilterResult
   │  filter_scanner_candidates_    │     (consumibles en F8 para
   │  with_radar(...)               │      paridad shadow / journal)
   └─────┬──────────────────────────┘
         │
         ▼ (RadarClient F6, httpx)
   ┌────────────────┐
   │ atlas_adapter  │  /api/radar/opportunities
   │   Radar (F5)   │
   └────────────────┘
```

## 9. Plan de rollback

Commit atómico local: `feat: F7 add scanner→Radar shadow filter service`.

Para revertir:

1. `git revert <commit-f7>` (commit atómico, sólo añade archivos
   nuevos y modifica `legacy_flags.py`).
2. Alternativa quirúrgica:
   - eliminar `atlas_code_quant/intake/scanner_radar_filter.py`,
   - eliminar `tests/atlas_code_quant/test_scanner_radar_filter_f7.py`,
   - eliminar `docs/ATLAS_CODE_QUANT_F7_SCANNER_RADAR_FILTER_SHADOW.md`,
   - revertir el bloque del flag `ATLAS_RADAR_FILTER_SHADOW_ENABLED`
     en `legacy_flags.py` (incluyendo entrada en `__all__`).
3. Validar `pytest tests/atlas_code_quant -q` (los 15 tests F7
   desaparecen; F6 mantiene 22 verdes).

Ningún cambio F7 toca runtime productivo, así que el rollback se
limita a borrar código durmiente y restaurar el archivo de flags. No
hay efecto sobre paper, live, scanner, Tradier ni Radar.

## 10. Próximas fases (referencia)

- **F8**: introducir el *consumer shadow* explícito (probable módulo
  en `monitoring/` o `operations/sensor_*` con su propio hook
  controlado), comparar paridad scanner-vs-Radar, journal interno y
  métricas Prometheus. Sigue sin enrutar tráfico real al Radar.
- **F9**: paridad estable → introducir flag
  `ATLAS_RADAR_FILTER_ENFORCED` y permitir gate duro por estrategia
  con rollback rápido.
- **F10**: deprecación final del scanner como fuente directa hacia
  estrategias.

Cada una de esas fases requiere su propio documento, tests dedicados,
commit atómico y aprobación humana explícita antes de cualquier push.
