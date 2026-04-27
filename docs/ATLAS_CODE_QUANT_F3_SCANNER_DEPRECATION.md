# F3 — Deprecación visible de endpoints scanner legacy

> **Fase:** F3 (Atlas Code Quant Refactor)
> **Estado:** completado, pendiente aprobación humana antes de F4
> **Commit:** `refactor: F3 add HTTP deprecation headers to legacy scanner endpoints`
> **Push:** NO autorizado en F3 (commit local únicamente)
> **Riesgo runtime:** mínimo — sólo se añaden headers HTTP y un log WARNING; no se altera status code, body ni schema.

---

## 1. Objetivo

Marcar como **deprecated** los 10 endpoints HTTP del scanner legacy (5 v1 + 5 v2) en `atlas_code_quant/api/main.py` mediante:

1. Headers HTTP estándar de deprecación (`Deprecation`, `Sunset`, `Link`).
2. Log estructurado a nivel `WARNING` por cada hit.
3. Marca `deprecated=True` en el decorador FastAPI (visible en OpenAPI/Swagger).
4. Documentación de la flag `ATLAS_LEGACY_SCANNER_ENABLED` (sin activar lógica condicional).

F3 **NO** ejecuta el cutover. Los endpoints siguen 100% operativos y devuelven exactamente el mismo contrato que antes.

## 2. Reglas duras respetadas

* ✅ Sólo se modifica `atlas_code_quant/api/main.py` y `docs/`.
* ✅ NO se tocan: Radar, intake, scanner wiring, execution/Tradier, operations, journal, vision, autonomy, FSM, locks paper/live, kill switch, guardrails de `operation_center`/`production_guard`.
* ✅ NO cambia status code, cuerpo ni schema de respuesta.
* ✅ NO añade validaciones nuevas.
* ✅ Helper se invoca **antes** de `_auth(...)` en v1: los headers de deprecación se estampan incluso si la llamada no está autenticada.
* ✅ NO push: commit local únicamente.

## 3. Cambios

### 3.1 Helper y constantes (`atlas_code_quant/api/main.py`, ~líneas 158–203)

```python
_SCANNER_DEPRECATION_SUNSET: str = "2026-12-31T23:59:59Z"
_SCANNER_DEPRECATION_DOC_LINK: str = '</docs/scanner-legacy>; rel="deprecation"'
_scanner_deprecation_logger = logging.getLogger("quant.api.deprecation.scanner")


def _apply_scanner_deprecation_headers(
    response: "Response",
    *,
    endpoint: str,
    api_version: str,
) -> None:
    response.headers["Deprecation"] = "true"
    response.headers["Sunset"] = _SCANNER_DEPRECATION_SUNSET
    response.headers["Link"] = _SCANNER_DEPRECATION_DOC_LINK
    _scanner_deprecation_logger.warning(
        "deprecated scanner endpoint hit",
        extra={
            "event": "scanner_endpoint_deprecated",
            "endpoint": endpoint,
            "api_version": api_version,
            "sunset": _SCANNER_DEPRECATION_SUNSET,
            "replacement": "/api/radar/* (institutional Radar)",
        },
    )
```

* `Deprecation: true` — RFC 8594 / draft-ietf-httpapi-deprecation-header.
* `Sunset: 2026-12-31T23:59:59Z` — fecha simbólica autorizada por el equipo (sólo señal, no enforcement).
* `Link: </docs/scanner-legacy>; rel="deprecation"` — apunta a la URL documental relativa para que clientes puedan localizar la guía.
* El logger dedicado `quant.api.deprecation.scanner` emite un WARNING estructurado con `event`, `endpoint`, `api_version`, `sunset` y `replacement`. Esto facilita filtrar uso legacy en observabilidad antes del cutover.

### 3.2 Endpoints afectados — rutas y líneas

| # | Ruta | Versión | Handler | Línea (post-F3) |
|---|---|---|---|---|
| 1 | `GET /scanner/status` | v1 | `scanner_status` | 3411 |
| 2 | `GET /scanner/report` | v1 | `scanner_report` | 3441 |
| 3 | `POST /scanner/config` | v1 | `scanner_config` | 3471 |
| 4 | `POST /scanner/control` | v1 | `scanner_control` | 3505 |
| 5 | `GET /scanner/universe/search` | v1 | `scanner_universe_search` | 3557 |
| 6 | `GET /api/v2/quant/scanner/status` | v2 | `scanner_status_v2` | 3803 |
| 7 | `GET /api/v2/quant/scanner/report` | v2 | `scanner_report_v2` | 3820 |
| 8 | `POST /api/v2/quant/scanner/config` | v2 | `scanner_config_v2` | 3840 |
| 9 | `POST /api/v2/quant/scanner/control` | v2 | `scanner_control_v2` | 3858 |
| 10 | `GET /api/v2/quant/scanner/universe/search` | v2 | `scanner_universe_search_v2` | 3876 |

**Patrón v1** (ejemplo `/scanner/status`):

```python
@app.get(
    "/scanner/status",
    response_model=StdResponse,
    tags=["Scanner"],
    deprecated=True,                       # nuevo en F3
)
async def scanner_status(
    response: Response,                    # nuevo en F3
    x_api_key: str | None = Header(None),
):
    """Estado operativo del escáner permanente de oportunidades.

    .. deprecated:: F3
        Use ``/api/radar/*`` ...
    """
    _apply_scanner_deprecation_headers(    # nuevo en F3 (antes de _auth)
        response, endpoint="/scanner/status", api_version="v1"
    )
    _auth(x_api_key)
    # ... cuerpo original sin cambios ...
```

**Patrón v2** (alias de v1, doble estampado):

```python
@app.get(
    "/api/v2/quant/scanner/status",
    response_model=StdResponse,
    tags=["V2"],
    deprecated=True,                       # nuevo en F3
)
async def scanner_status_v2(
    response: Response,                    # nuevo en F3
    x_api_key: str | None = Header(None),
):
    """.. deprecated:: F3 alias v2 de ``/scanner/status``; usar Radar institucional."""
    _apply_scanner_deprecation_headers(    # nuevo en F3 (api_version="v2")
        response, endpoint="/api/v2/quant/scanner/status", api_version="v2"
    )
    return await scanner_status(response=response, x_api_key=x_api_key)
```

> Por qué v2 estampa primero con `api_version="v2"` y luego pasa el mismo `response` al handler v1: el log discrimina correctamente quién entró por v2, y los headers finales reflejan los del helper más reciente (mismos valores, idempotente). El cuerpo de respuesta sigue delegando 100% a v1 → sin riesgo de drift semántico.

### 3.3 Flag `ATLAS_LEGACY_SCANNER_ENABLED` (`atlas_code_quant/config/legacy_flags.py`)

```python
ATLAS_LEGACY_SCANNER_ENABLED: bool = True
```

* Default `True`: endpoints siguen activos.
* **Documental en F3**: `api/main.py` no la lee como condicional. Sólo se anota su contrato público para que la fase de cutover (post-F4) pueda usarla para devolver `410 Gone` o redirigir a `/api/radar/*`.
* Las reglas duras (NO usarla aún para deshabilitar lógica, NO cambiar default sin coordinación) están documentadas en el módulo.

### 3.4 Tests (`atlas_code_quant/tests/test_scanner_deprecation_headers.py`)

65 tests parametrizados que verifican vía AST estructural sin requerir cargar la app FastAPI completa (la app tiene dependencias preexistentes —`sqlalchemy`, `backtesting`— no instaladas en este sandbox; ortogonal a F3):

* Constantes y helper a nivel módulo definidos correctamente.
* Helper escribe los 3 headers requeridos.
* Helper emite el WARNING estructurado con el evento esperado.
* Para los 10 handlers:
  * Decorador con `deprecated=True`.
  * Firma con `response: Response`.
  * Llamada al helper con `endpoint` y `api_version` correctos.
  * (Sólo v1) helper invocado **antes** de `_auth(...)`.
  * (Sólo v2) sigue delegando en su par v1 (no cambia semántica).
  * Body sigue devolviendo `StdResponse` (directa o vía delegación).
* `ATLAS_LEGACY_SCANNER_ENABLED` documentada y exportada.

Resultado: **65 passed in 0.20s**.

## 4. Ejemplo de respuesta HTTP (especificación)

Cualquier llamada a un endpoint legacy responde con cuerpo idéntico al pre-F3 más estos headers añadidos:

```
HTTP/1.1 200 OK
Content-Type: application/json
Deprecation: true
Sunset: 2026-12-31T23:59:59Z
Link: </docs/scanner-legacy>; rel="deprecation"

{"ok": true, "data": {...}, "ms": 12.34}
```

Y un log WARNING como:

```json
{
  "level": "WARNING",
  "logger": "quant.api.deprecation.scanner",
  "msg": "deprecated scanner endpoint hit",
  "event": "scanner_endpoint_deprecated",
  "endpoint": "/scanner/status",
  "api_version": "v1",
  "sunset": "2026-12-31T23:59:59Z",
  "replacement": "/api/radar/* (institutional Radar)"
}
```

## 5. Aclaraciones explícitas

* F3 **NO** desactiva el scanner.
* F3 **NO** cambia status code, cuerpo, schema ni validaciones.
* F3 **NO** rompe ningún consumidor existente. Clientes que ignoren los headers `Deprecation/Sunset/Link` siguen funcionando idénticamente.
* F3 **NO** modifica las rutas Radar (`/api/radar/*`), intake, execution Tradier, journal, autonomy, FSM, locks paper/live, kill switch, ni guardrails.
* F3 **NO** lee la flag `ATLAS_LEGACY_SCANNER_ENABLED` como condicional. Sólo la documenta.
* F3 **NO** ejecuta cutover. La fecha `Sunset: 2026-12-31` es señal informativa, no enforcement.

## 6. Impacto en consumidores

Consumidores conocidos del scanner (no se tocan en F3, ver `atlas_code_quant/legacy/README_SCANNER_FREEZE.md`):

* `atlas_code_quant/api/main.py` — los propios endpoints (única edición en F3).
* `atlas_code_quant/backtester.py:27`.
* `atlas_code_quant/learning/trading_implementation_scorecard.py:568`.
* `atlas_code_quant/tests/test_scanner*.py`.
* Compat shim `scanner/__init__.py`.

Acción para clientes externos: empezar a registrar en su capa HTTP el header `Deprecation: true` y planificar migración a Radar institucional antes del `Sunset`.

## 7. Guía de migración a Radar (alto nivel — NO implementado en F3)

| Capacidad legacy (`/scanner/*`) | Sucesor canónico (Radar) |
|---|---|
| `GET /scanner/status` | Endpoint Radar de estado del motor (definido en F5+). |
| `GET /scanner/report` | Vista Radar de candidatos/rechazos/actividad. |
| `POST /scanner/config` | Configuración de Radar / universe provider. |
| `POST /scanner/control` | Control del loop Radar (start/stop/cycle). |
| `GET /scanner/universe/search` | Universe provider del Radar. |

> El mapping concreto, contratos Pydantic y SSE Radar se definirán y testean en fases posteriores (F5+). Esta tabla es orientativa y NO compromete el cutover.

## 8. Criterio de aceptación F3

1. ✅ Helper `_apply_scanner_deprecation_headers` definido a nivel módulo, con firma `(response, *, endpoint, api_version)`.
2. ✅ Los 3 headers (`Deprecation: true`, `Sunset`, `Link`) presentes en todas las respuestas legacy.
3. ✅ Decorador `deprecated=True` aplicado a los 10 endpoints (visible en OpenAPI/Swagger).
4. ✅ WARNING estructurado emitido con `event=scanner_endpoint_deprecated`.
5. ✅ Status code, body y schema idénticos al pre-F3 (verificado por test estructural y por delegación v2→v1 inalterada).
6. ✅ Helper invocado **antes** de `_auth(...)` en v1.
7. ✅ Flag `ATLAS_LEGACY_SCANNER_ENABLED` documentada en `legacy_flags.py` (default `True`, doc-only).
8. ✅ Tests F3: 65/65 passed.
9. ✅ Sin regresión de colección: 1033 tests collected (968 baseline F2 + 65 F3), mismos 40 errores preexistentes (sqlalchemy/backtesting) — no es regresión F3.
10. ✅ Commit atómico F3 sin push.

## 9. Plan de rollback

F3 es estructuralmente reversible y no toca runtime crítico. Para revertir:

```bash
git -C /home/user/workspace/atlas-core revert <hash-commit-F3>
```

Esto restaura los 10 handlers a su versión pre-F3 (sin `deprecated=True`, sin parámetro `response`, sin llamada al helper) y elimina helper, constantes, tests y la flag `ATLAS_LEGACY_SCANNER_ENABLED`. La doc puede mantenerse como histórico o eliminarse junto al revert.

## 10. Referencias

* `atlas_code_quant/api/main.py` (helper líneas ~158–203; endpoints líneas 3411–3888)
* `atlas_code_quant/config/legacy_flags.py` (flag `ATLAS_LEGACY_SCANNER_ENABLED`)
* `atlas_code_quant/tests/test_scanner_deprecation_headers.py` (65 tests)
* `atlas_code_quant/legacy/README_SCANNER_FREEZE.md` (freeze del scanner desde F1)
* `docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md` (F0)
* `docs/ATLAS_CODE_QUANT_F1_REORG.md` (F1)
* `docs/ATLAS_CODE_QUANT_F2_INTERNAL_SIMULATOR_RENAME.md` (F2)
