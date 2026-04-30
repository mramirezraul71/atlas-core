# Informe v3 — Gate liviano real (`status_for_gate`), semántica ampliada, tests HTTP, timeout bridge

**Repo:** `C:\ATLAS_PUSH` · **Fecha:** 2026-04-11  

## Contexto (respuesta a auditoría externa post-v2)

La v2 enlazó `StdResponse.ok` con `data.ready` y eliminó el import frágil de `chart_plan`. Quedaba deuda: el “fast” seguía usando `status(fast=False)` (incluía sonda Insta360 aunque el proveedor fuera `direct_nexus`), el readiness era estrecho (solo browser + visión), y los tests no cubrían el wiring HTTP ni Yahoo/edge en el builder.

## Cambios v3 (implementados en esta iteración)

### 1. Modo gate de visión (`SensorVisionService.status_for_gate`)
- Solo ejecuta comprobaciones del **proveedor activo** (no `_insta360_available()` si el modo es `direct_nexus`).
- `direct_nexus`: health con timeout acotado (**1,5 s** por defecto).
- **`atlas_push_bridge`**: `_atlas_push_bridge_status(timeout_sec=…)` con **2 s por defecto** en gate (no el timeout operativo largo de PUSH); capturas y `diagnose()` siguen usando el timeout completo.

### 2. `GET /operation/readiness` usa el gate
- Sustituye `_VISION.status(fast=False)` por **`_VISION.status_for_gate()`**.

### 3. Semántica de readiness ampliada (`readiness_eval.py`)
- **`chart_plan_probe_ok`** (`chart_plan_builder.py`): valida plan con 3 targets y URLs `https://` cuando aplica flujo de charts (`chart_auto_open` **o** `startup_chart_warmup`).
- **`startup_warmup_gate_satisfied`**: si warmup + auto-open están activos, exige evidencia en `chart_execution.last_payload` (`open_ok` o `execution_state` en `opened` / `cooldown_reuse`).
- **Diagnóstico**: `visual_pipeline_ok` falso si el dict de pipeline incluye clave `"error"` (fallo al instanciar `VisualPipeline`).
- Payload fast/diagnostic incluye **`chart_plan_probe`** y **`startup_warmup_gate`**.

### 4. Tests
- `test_sensor_vision_status.py`: gate sin Insta360 en `direct_nexus`; **bridge usa `timeout_sec` corto** en `_request_json`.
- `test_readiness_eval.py`: `chart_plan_buildable`, `visual_pipeline_ok`, warmup gate.
- `test_chart_plan_builder.py`: Yahoo, TF superior por defecto, símbolo vacío/espacios.
- **`test_operation_readiness_http.py`**: `TestClient` + mock de payload → **`ok` alineado con `data.ready`**; 401 sin API key.

## Rutas (actualizado)

| Ruta | Uso |
|------|-----|
| `GET /operation/readiness` | Gate rápido: `status_for_gate` + reglas readiness ampliadas |
| `GET /operation/readiness/diagnostic` | `diagnose()` + `VisualPipeline.status()` + mismas reglas + pipeline sin error |
| Espejos `/api/v2/quant/operation/readiness*` | Igual |

## Verificación ejecutada (esta sesión)

```powershell
cd C:\ATLAS_PUSH
.\venv\Scripts\python.exe -m pytest atlas_code_quant/tests/test_readiness_eval.py `
  atlas_code_quant/tests/test_chart_plan_builder.py atlas_code_quant/tests/test_sensor_vision_status.py `
  atlas_code_quant/tests/test_operation_readiness_http.py atlas_code_quant/tests/test_startup_visual_connect.py -q
```

```powershell
# Quant API (puerto configurado en settings, ej. 8795). No confundir con PUSH :8791 (sin rutas /operation/readiness).
.\venv\Scripts\python.exe scripts\quant_reality_check.py --http http://127.0.0.1:8795
```

**Resultado:** pytest (lotes anteriores) OK; `quant_reality_check --http http://127.0.0.1:8795` **exit 0** en verificación local.

**Nota de despliegue:** hasta reiniciar uvicorn con el código de este commit, en JSON puede verse un `vision_status` antiguo. Con la v3 desplegada, en modo fast debe aparecer **`vision_status.status_mode == "gate"`**.

## Commit local v3

**Mensaje de commit:** `fix(quant): readiness v3 gate (status_for_gate), probe charts/warmup, bridge timeout corto, tests HTTP`  

**SHA:** obtener con `git log -1 --oneline` en la rama activa (el hash cambia si se hace `commit --amend`).  

## Archivos tocados (ámbito v3)

- `atlas_code_quant/operations/sensor_vision.py` — `status_for_gate`, timeout bridge/nexus  
- `atlas_code_quant/operations/chart_plan_builder.py` — `chart_plan_probe_ok`  
- `atlas_code_quant/operations/readiness_eval.py` — reglas + `startup_warmup_gate_satisfied`  
- `atlas_code_quant/api/main.py` — payloads fast/diagnostic  
- `atlas_code_quant/tests/test_*.py` — cobertura ampliada  

*Fin informe v3.*
