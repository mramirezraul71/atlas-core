# Atlas Code Quant — F10: scanner subordinado a Radar (enforcement API-level)

> **Estado:** F10 implementada, gobernada por flag y testeada.
> **Default:** `ATLAS_RADAR_FILTER_ENFORCED = False` (modo compat).
> **No push.** Activar enforcement requiere aprobación humana explícita.

---

## 1. Contexto y política

Tras F5–F9 el repositorio dispone de:

* Radar multi-símbolo canónico (F5):
  `GET /api/radar/opportunities`, `…/{symbol}`, `…/stream`.
* Code Quant intake / shadow (F6/F7/F8/F9):
  - `RadarClient` y modelos internos,
  - `filter_scanner_candidates_with_radar` (F7),
  - `run_scanner_radar_shadow` (F8) + reporte agregado,
  - `maybe_run_scanner_radar_shadow` (F9) gated por
    `ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`.

**Política de negocio formalizada en F10:**

> **scanner propone, Radar decide.**
>
> El scanner sigue siendo motor de exploración (recall). Pero todo lo
> que sale al mundo como "oportunidad" oficial pasa por el gate Radar
> (precision). Nada llega a estrategias ni a clientes como
> *oportunidad* sin sello Radar.

F10 codifica esa política **en el plano API** sobre el endpoint
`/scanner/report` (y su alias v2), gobernado por la flag dura
`ATLAS_RADAR_FILTER_ENFORCED`.

---

## 2. Alcance F10

### 2.1 Tocado

| Archivo | Cambio | Tamaño |
|---|---|---|
| `atlas_code_quant/intake/scanner_radar_view.py` | nuevo adapter `build_scanner_report_via_radar` | ~380 líneas |
| `atlas_code_quant/api/main.py` | `/scanner/report` consume el adapter; import añadido | ~28 líneas |
| `tests/atlas_code_quant/test_scanner_radar_view_f10.py` | 22 tests F10 | nuevo |
| `docs/ATLAS_CODE_QUANT_F10_SCANNER_SUBORDINATED_TO_RADAR_ENFORCEMENT.md` | esta documentación | nuevo |

### 2.2 NO tocado (alcance prohibido respetado)

* `atlas_code_quant/execution/*` (Tradier, broker_router, signal_executor)
* `atlas_code_quant/operations/*` runtime live/paper loops
* `atlas_code_quant/autonomy/*`, `risk/*`, `vision/*`,
  `production_guard/*`
* `atlas_adapter/*` (Radar canónico desde F5)
* `monitoring/scanner_radar_shadow.py` (invariante F8)
* `monitoring/scanner_radar_shadow_hook.py` (contrato F9)
* Locks `paper_only`, `full_live_globally_locked`, dry-run defaults
* Endpoints scanner restantes: `/scanner/status`, `/scanner/config`,
  `/scanner/control`, `/scanner/universe/search`. Sus headers de
  deprecación F3, su semántica y sus cuerpos no se alteran.

---

## 3. Adapter `build_scanner_report_via_radar`

Ubicación: `atlas_code_quant/intake/scanner_radar_view.py`.

### 3.1 Firma

```python
def build_scanner_report_via_radar(
    scanner_report: Any,
    *,
    radar_client: RadarClient | None = None,
    min_score: float = DEFAULT_MIN_SCORE,  # 70.0
    enforced: bool | None = None,
) -> dict[str, Any]: ...
```

### 3.2 Pipeline interno

1. **Normaliza** la salida del scanner (`OpportunityScannerService.report(...)`)
   a un dict con shape compatible con `ScannerReportPayload`. Tolera
   inputs ``None`` / no-mapping → esqueleto vacío.
2. **Filtra** los candidatos del scanner contra el Radar reusando
   `filter_scanner_candidates_with_radar` (F7) — única lógica de mapping
   shadow/enforce.
3. **Aplica modo** según la flag:
   * **compat**: deja `data.candidates` tal cual el scanner; añade
     campos Radar al lado.
   * **enforced**: `data.candidates` queda **sólo** con
     `approved_by_radar`. La lista raw del scanner se preserva en
     `data.scanner_raw_candidates` para auditoría.
4. **Empaqueta** el resultado con metadata Radar (`radar_meta`)
   incluyendo `mode`, `enforced`, `policy`, `batch_failed`,
   `min_score`, conteos, `trace_id`, `radar_source`.

### 3.3 Política de aceptación (heredada de F7)

| Condición Radar | Resultado |
|---|---|
| `score ≥ 70.0` y `classification ∈ {high_conviction, watchlist}` | **approved** |
| `score < 70.0` | rejected |
| `classification == "reject"` | rejected (incluso con score alto) |
| Sin oportunidad para el símbolo | rejected (`no_data`) |

### 3.4 Política de degradación

Códigos de batch (`RADAR_TIMEOUT`, `RADAR_UNREACHABLE`, `RADAR_HTTP_ERROR`,
`RADAR_INVALID_PAYLOAD`):

| Modo | Comportamiento |
|---|---|
| **compat** | `data.candidates` conserva la lista del scanner. `radar_degradations` se reporta honestamente. `radar_meta.batch_failed = True`. |
| **enforced** | **Política conservadora**: `data.candidates = []`. NO se promueve ningún candidato del scanner si Radar está caído. `radar_meta.policy = "conservative_block_on_degradation"`. La lista raw queda preservada en `scanner_raw_candidates`. |

> **Decisión explícita F10:** preferimos quedarnos sin oportunidades
> que devolver oportunidades sin sello Radar. Eso es coherente con
> "scanner propone, Radar decide".

El adapter **nunca lanza** al caller. La red de seguridad de F7 ya
garantiza que el filtro no propaga excepciones; el handler de
`/scanner/report` mantiene su `try/except` original como cinturón.

---

## 4. Flag `ATLAS_RADAR_FILTER_ENFORCED`

Definida desde F9 como **doc-only**; F10 la convierte en **flag de
runtime efectiva** consultada por
`build_scanner_report_via_radar` (vía `getattr(legacy_flags, ...)` para
permitir `monkeypatch` en tests).

* **Default:** `False`.
* **Lectura dinámica:** la flag se consulta cada llamada — un cambio
  toma efecto en la siguiente petición.
* **No se consulta** desde execution / autonomy / risk / vision /
  production_guard / atlas_adapter (verificado por test estático
  `test_adapter_does_not_import_forbidden_modules`).
* **Activación:** modificar el valor en `legacy_flags.py` (commit
  dedicado) o setearla por código de orquestación humana. F10 NO la
  promueve a `True` automáticamente.

---

## 5. Cambio en `/scanner/report` (api/main.py)

### 5.1 Diff resumido

**Imports:** añadido
```python
from atlas_code_quant.intake.scanner_radar_view import (
    build_scanner_report_via_radar,
)
```

**Handler:** dentro del `try` reemplazamos
```python
payload = ScannerReportPayload.model_validate(_SCANNER.report(activity_limit=...))
return StdResponse(ok=True, data=payload.model_dump(), ms=...)
```
por
```python
scanner_raw = _SCANNER.report(activity_limit=activity_limit)
gated = build_scanner_report_via_radar(scanner_raw)
payload = ScannerReportPayload.model_validate(gated)
data = payload.model_dump()
for extra_key in (
    "radar_filtered_candidates",
    "radar_rejected",
    "radar_degradations",
    "radar_meta",
    "scanner_raw_candidates",
):
    if extra_key in gated:
        data[extra_key] = gated[extra_key]
return StdResponse(ok=True, data=data, ms=...)
```

### 5.2 Invariantes preservadas

* Status code: 200 igual que antes.
* Body wrapper: `StdResponse{ok, data, error, ms}`.
* Headers F3: `Deprecation`, `Sunset`, `Link` siguen estampándose
  ANTES de `_auth(...)` (test AST F3 verde).
* Cuerpo del payload: `ScannerReportPayload.model_validate(...)` sigue
  ejecutándose sobre el dict resultante, garantizando los campos
  heredados (`generated_at`, `status`, `summary`, `criteria`,
  `universe`, `candidates`, `rejections`, `activity`, `current_work`,
  `learning`, `error`).
* `scanner_report_v2` sigue delegando 1:1 en `scanner_report` →
  enforcement F10 se hereda automáticamente para el alias v2.

### 5.3 No tocado

* `/scanner/status`, `/scanner/config`, `/scanner/control`,
  `/scanner/universe/search`: sin cambios. Su semántica F3 se
  preserva. (F10 puede añadir telemetría Radar a `/scanner/status`
  en una fase posterior si se considera necesario; no es alcance F10.)

---

## 6. Shape antes / después de `/scanner/report`

### 6.1 Antes (pre-F10)

```jsonc
{
  "ok": true,
  "data": {
    "generated_at": "2026-04-27T12:00:00+00:00",
    "status": {...},
    "summary": {...},
    "criteria": [...],
    "universe": {...},
    "candidates": [
      {"symbol": "AAPL", "selection_score": 81.0, ...},
      {"symbol": "MSFT", "selection_score": 60.0, ...}
    ],
    "rejections": [...],
    "activity": [...],
    "current_work": {...},
    "learning": {...},
    "error": null
  },
  "ms": 12.4
}
```

### 6.2 Después — modo compat (default, `ATLAS_RADAR_FILTER_ENFORCED=False`)

```jsonc
{
  "ok": true,
  "data": {
    "generated_at": "...",
    "status": {...},
    "summary": {...},
    "criteria": [...],
    "universe": {...},
    "candidates": [
      // SIN cambios respecto a pre-F10. La lista oficial sigue siendo el scanner.
      {"symbol": "AAPL", "selection_score": 81.0, ...},
      {"symbol": "MSFT", "selection_score": 60.0, ...}
    ],
    "rejections": [...],
    "activity": [...],
    "current_work": {...},
    "learning": {...},
    "error": null,

    // ───── Campos NUEVOS F10 ─────
    "radar_filtered_candidates": [
      {"symbol": "AAPL", "selection_score": 81.0,
       "radar_decision": {"approved": true, "classification": "high_conviction",
                          "score": 85.0, "trace_id": "..."}}
    ],
    "radar_rejected": [
      {"symbol": "MSFT", "approved": false, "classification": "reject",
       "score": 40.0, "degradations": []}
    ],
    "radar_degradations": [],
    "radar_meta": {
      "mode": "compat",
      "enforced": false,
      "policy": "compat_observation_only",
      "min_score": 70.0,
      "scanner_input": 2,
      "approved_count": 1,
      "rejected_count": 1,
      "batch_failed": false,
      "trace_id": "trace-batch-...",
      "timestamp": "2026-04-27T12:00:00+00:00",
      "radar_source": "quant"
    }
  },
  "ms": 18.7
}
```

### 6.3 Después — modo enforced (`ATLAS_RADAR_FILTER_ENFORCED=True`)

```jsonc
{
  "ok": true,
  "data": {
    "generated_at": "...",
    "status": {...},
    "summary": {...},
    "criteria": [...],
    "universe": {...},
    "candidates": [
      // ↓ SOLO aprobados por Radar.
      {"symbol": "AAPL", "selection_score": 81.0,
       "radar_decision": {"approved": true, "classification": "high_conviction",
                          "score": 85.0, "trace_id": "..."}}
    ],
    "rejections": [...],
    "activity": [...],
    "current_work": {...},
    "learning": {...},
    "error": null,

    // ───── Campos NUEVOS F10 ─────
    "scanner_raw_candidates": [
      // Lista original del scanner, antes del gate. Solo en enforced.
      {"symbol": "AAPL", "selection_score": 81.0, ...},
      {"symbol": "MSFT", "selection_score": 60.0, ...}
    ],
    "radar_filtered_candidates": [/* idéntico a candidates */],
    "radar_rejected": [/* MSFT etc. */],
    "radar_degradations": [],
    "radar_meta": {
      "mode": "enforced",
      "enforced": true,
      "policy": "radar_gate_strict",
      "min_score": 70.0,
      "scanner_input": 2,
      "approved_count": 1,
      "rejected_count": 1,
      "batch_failed": false,
      ...
    }
  }
}
```

### 6.4 Después — enforced + Radar degradado (5xx / timeout / payload inválido)

```jsonc
{
  "ok": true,
  "data": {
    "generated_at": "...",
    "candidates": [],                       // ← política conservadora
    "rejections": [...],
    ...
    "scanner_raw_candidates": [/* preservado para auditoría */],
    "radar_filtered_candidates": [],
    "radar_rejected": [/* todos como no_data */],
    "radar_degradations": [
      {"code": "RADAR_HTTP_ERROR", "label": "...", "severity": "warning", "source": "..."}
    ],
    "radar_meta": {
      "mode": "enforced",
      "enforced": true,
      "policy": "conservative_block_on_degradation",
      "batch_failed": true,
      ...
    }
  }
}
```

---

## 7. Tests F10

Archivo: `tests/atlas_code_quant/test_scanner_radar_view_f10.py` (22 tests).

| Categoría | Tests |
|---|---|
| Modo compat | `test_default_flag_is_false`, `test_compat_keeps_official_candidates_intact`, `test_compat_preserves_other_scanner_fields`, `test_compat_preserves_scanner_when_radar_degraded` |
| Modo enforced | `test_enforced_official_candidates_only_radar_approved`, `test_enforced_reads_runtime_flag`, `test_enforced_does_not_promote_when_radar_returns_no_data` |
| Degradaciones | `test_enforced_blocks_all_when_radar_degraded[timeout/http_5xx/invalid_payload]` |
| Defensivo | `test_handles_none_input_gracefully`, `test_handles_non_dict_input_gracefully`, `test_handles_empty_candidates`, `test_does_not_raise_when_filter_explodes` (defense-in-depth con mock) |
| Schema compat | `test_compat_payload_validates_against_scanner_report_schema`, `test_enforced_payload_validates_against_scanner_report_schema` |
| AST guards `api/main.py` | `test_api_main_imports_adapter`, `test_scanner_report_handler_invokes_adapter`, `test_scanner_report_v2_still_delegates_to_v1`, `test_scanner_report_handler_returns_stdresponse` |
| Aislamiento | `test_adapter_does_not_import_forbidden_modules`, `test_adapter_consults_enforcement_flag_textually` |

**Resultado:** 22/22 verdes localmente.

**Regresión:**
* F3 deprecation tests (65): verdes.
* F6/F7/F8/F9 tests (72): verdes.
* Total shadow + F10: **159/159 verdes**.
* Baseline `python -m pytest atlas_code_quant/tests --collect-only -q`:
  **1054 tests / 40 errores preexistentes** — sin regresión de
  entorno.

---

## 8. Confirmación explícita de invariantes

* No se toca `execution/*`, Tradier, broker_router, live loops.
* No se toca `atlas_adapter/*`.
* No se toca autonomy / risk / vision / locks / production_guard.
* No se introduce live trading.
* `paper_only=True`, `full_live_globally_locked=True` y dry-run
  defaults intactos.
* `ATLAS_RADAR_FILTER_ENFORCED` queda en `False` por defecto:
  comportamiento observable de `/scanner/report` para el cliente
  no cambia hasta que un humano flippe la flag.
* El scanner sigue disponible como módulo de exploración interna
  (`backtester.py`, `notifications/payloads.py`); F10 sólo subordina
  la **API pública de oportunidades**.
* La invariante F8
  (`test_no_runtime_module_consults_runtime_flag`) sigue verde:
  `monitoring/scanner_radar_shadow.py` no se modificó.
* Los AST tests F3
  (`test_v1_handler_calls_helper_before_auth`,
  `test_v2_handler_delegates_to_v1`,
  `test_handler_response_body_unchanged`) siguen verdes para el
  handler `scanner_report` modificado.

---

## 9. Rollback

Commit F10:

```
feat: F10 enforce Radar as gate for scanner opportunities (API-level)
```

**Archivos tocados:**

* `atlas_code_quant/intake/scanner_radar_view.py` (nuevo)
* `atlas_code_quant/api/main.py` (modificado: import + handler `/scanner/report`)
* `tests/atlas_code_quant/test_scanner_radar_view_f10.py` (nuevo)
* `docs/ATLAS_CODE_QUANT_F10_SCANNER_SUBORDINATED_TO_RADAR_ENFORCEMENT.md` (nuevo)

**Rollback inmediato:**

```bash
git revert <SHA-F10>
```

Tras el revert, `/scanner/report` vuelve al comportamiento pre-F10
(scanner puro). Los módulos shadow F6/F7/F8/F9 siguen disponibles —
sólo el adapter API-level desaparece.

**Rollback parcial sin revert** (si se quisiera mantener el adapter
disponible pero desactivar el cambio en API): basta con dejar en
False la flag (default) — el shape oficial de `data.candidates` no
cambia, y el cliente sólo recibe los campos Radar adicionales
(opcionales y aditivos). Para volver al payload pre-F10 *exacto*
hay que revertir el commit.

---

## 10. Próximos pasos sugeridos (fuera de F10)

1. **Activación gradual:** monitorear `radar_meta.batch_failed`,
   `radar_meta.divergence` y registrar diferencia compat-vs-enforced
   sobre tráfico real durante un periodo razonable antes de flippear
   la flag.
2. **Endpoint Radar-first:** considerar exponer `/api/radar/opportunities`
   como ruta primaria y dejar `/scanner/report` como wrapper delgado
   (puramente de transición). F10 deja la base para esto.
3. **Enforcement interno** (consumidores no-API: backtester,
   notifications/payloads): fase dedicada con su propio plan de
   cutover y paridad. F10 explícitamente NO entra ahí.
4. **Limpieza F3:** una vez estable el enforcement, retirar
   `/scanner/*` siguiendo la fecha de Sunset (`2026-12-31`).
