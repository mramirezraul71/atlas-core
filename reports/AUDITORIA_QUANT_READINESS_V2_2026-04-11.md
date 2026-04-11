# Informe v2 — Correcciones tras auditoría (readiness semántico, fast vs diagnostic, warmup estable)

**Repo:** `C:\ATLAS_PUSH` · **Fecha:** 2026-04-11  
**Commit local (`variante/nueva`):** `af83c2f88`  
**Commit GitHub (rama `push/quant-readiness-v2-20260411`):** `e7f806b8d`  
**PR:** https://github.com/mramirezraul71/atlas-core/pull/new/push/quant-readiness-v2-20260411  

## Qué se corrigió (respuesta a hallazgos previos)

### 1. Readiness ya no es un falso positivo por HTTP 200
- **`GET /operation/readiness`** (y v2) ahora es modo **fast**: arma `data.ready` y **`StdResponse.ok = data.ready`** con reglas explícitas en `operations/readiness_eval.py::evaluate_operational_readiness`:
  - Si `chart_auto_open_enabled` → exige `browser_available`.
  - Exige `vision_provider_ready` (desde `SensorVisionService.status(fast=False)`, sin `diagnose()`).
- El CLI **`scripts/quant_reality_check.py`** exige **`ok==true` y `data.ready==true`** en el JSON (`readiness_http_body_ok`), no solo status HTTP.

### 2. Diagnóstico profundo separado del gate liviano
- **`GET /operation/readiness/diagnostic`** (y v2): incluye `diagnose()` + `VisualPipeline.status()`; misma semántica **`ready`/`ok`** calculada sobre `diagnose["provider_ready"]`.
- El endpoint fast **omite** visual_pipeline con puntero explícito al diagnostic.

### 3. Warmup sin import frágil a `_chart_plan` privado
- **`operations/chart_plan_builder.py`**: `build_selector_chart_plan` como API estable.
- **`strategy_selector`** y **`startup_visual_connect`** importan desde ahí (con fallback `ModuleNotFoundError` donde aplica).

### 4. Tests ampliados
- `test_readiness_eval.py` — reglas `evaluate_operational_readiness` + `readiness_http_body_ok`.
- `test_chart_plan_builder.py` — URLs/targets.
- `test_startup_visual_connect.py` — **happy path** warmup llama `ensure_chart_mission` con plan real.

## Rutas

| Ruta | Uso |
|------|-----|
| `GET /operation/readiness` | Gate rápido (~vision status full, sin diagnose ni pipeline) |
| `GET /operation/readiness/diagnostic` | Observabilidad pesada |
| Espejos `/api/v2/quant/operation/readiness*` | Igual |

## Verificación local

```powershell
cd C:\ATLAS_PUSH
.\scripts\atlas_quant_start.ps1 -Port 8795 -MaxWaitSec 90
.\venv\Scripts\python.exe C:\ATLAS_PUSH\scripts\quant_reality_check.py --http http://127.0.0.1:8795
```

**Nota:** Si `sensor_vision_state.json` tiene `direct_nexus` y el robot :8002 está caído, **`ready=false`** es el comportamiento correcto hasta cambiar proveedor (`off`, `desktop_capture`, etc.).

## Archivos nuevos o relevantes

- `atlas_code_quant/operations/chart_plan_builder.py`
- `atlas_code_quant/operations/readiness_eval.py`
- Cambios en `api/main.py`, `selector/strategy_selector.py`, `operations/startup_visual_connect.py`, `scripts/quant_reality_check.py`, `config/atlas.env.example`, tests bajo `atlas_code_quant/tests/`.

*Fin informe v2.*
