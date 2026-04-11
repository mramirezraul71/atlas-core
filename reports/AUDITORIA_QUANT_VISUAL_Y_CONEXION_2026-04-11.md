# Informe para auditoría externa — Quant: readiness, gráficos, visión y conexión en arranque

**Repositorio:** `C:\ATLAS_PUSH` (atlas-core)  
**Rama de trabajo local:** `variante/nueva`  
**Fecha:** 2026-04-11  
**Objetivo del cambio:** Dejar comprobable y conectado el flujo visual (charts + cámara gobernada), endpoints de readiness, verificación CLI, y arranque del API alineado con StrategySelector.

---

## 1. Resumen ejecutivo

1. **Readiness HTTP:** `GET /operation/readiness` y espejo `GET /api/v2/quant/operation/readiness` devuelven agregado de chart execution, diagnóstico de visión, estado del visual pipeline y `quant_flags` (incl. flags de arranque y chart provider).
2. **Charts:** `ChartExecutionService.ensure_chart_mission` abre navegador solo si `QUANT_CHART_AUTO_OPEN_ENABLED=true`, hay `chart_plan.targets` con URLs, y hay ejecutable detectado; verificación opcional post-`Popen` (Windows `tasklist`).
3. **Conexión arranque:** `operations/startup_visual_connect.py` — al terminar el startup en background (también en lightweight), aplica `QUANT_DEFAULT_VISION_PROVIDER` al estado persistido y, si `QUANT_STARTUP_CHART_WARMUP=true` y auto-open activo, llama a `ensure_chart_mission` con el mismo `_chart_plan` que el StrategySelector.
4. **Selector alineado con visión:** `camera_plan["provider"]` pasa de fijo `direct_nexus` a `_active_vision_provider()` leyendo `SensorVisionService().status(fast=True)`.
5. **Auto-cycle:** `_SELECTOR.proposal(..., chart_provider=settings.chart_provider_default)` para coherencia con `QUANT_CHART_PROVIDER`.
6. **Verificación objetiva:** `scripts/quant_reality_check.py` — carga `config/atlas.env` antes de `config.settings`; con `--http` exige por defecto `/health` **y** readiness OK; `--http-health-only` relaja readiness.
7. **Tests:** `test_chart_execution_contract.py` (contrato ensure_chart_mission), `test_startup_visual_connect.py` (arranque conectado).

**Problema operativo resuelto:** Un proceso uvicorn antiguo en `:8795` devolvía 404 en readiness; **reinicio del Quant desde el repo** (parar PID en 8795 + `scripts\atlas_quant_start.ps1`) alinea binario con código y readiness pasa.

---

## 2. Archivos tocados (alcance de esta entrega)

| Ruta | Rol |
|------|-----|
| `atlas_code_quant/api/main.py` | `_operation_readiness_payload`, rutas readiness, `apply_startup_visual_connect` en `_start_background_services` (full + lightweight), auto-cycle `chart_provider_default`. |
| `atlas_code_quant/config/settings.py` | `default_vision_provider`, `startup_chart_warmup_*`, `chart_provider_default`, lista `startup_chart_warmup_symbols` en `__post_init__`. |
| `atlas_code_quant/operations/startup_visual_connect.py` | **Nuevo:** conexión visión + warmup charts. |
| `atlas_code_quant/selector/strategy_selector.py` | `_active_vision_provider()`, `camera_plan.provider` dinámico. |
| `atlas_code_quant/tests/test_chart_execution_contract.py` | **Nuevo:** contrato real de chart mission. |
| `atlas_code_quant/tests/test_startup_visual_connect.py` | **Nuevo:** smoke de startup connect. |
| `scripts/quant_reality_check.py` | **Nuevo:** CLI verificación; readiness obligatorio con `--http`. |
| `config/atlas.env.example` | Documentación variables Quant + comandos de verificación. |

**Commits previos en la misma línea temática (referencia):** `feat(quant): readiness flags, chart verify, regime/cycle hysteresis` — incluye `chart_execution`, regime hysteresis, price cycles, etc.

---

## 3. Variables de entorno relevantes

- `QUANT_CHART_AUTO_OPEN_ENABLED` — default false; sin true no hay `Popen` del navegador.
- `QUANT_STARTUP_CHART_WARMUP` + `QUANT_STARTUP_CHART_SYMBOLS` + `QUANT_STARTUP_CHART_TIMEFRAME` — warmup al arrancar API (requiere auto-open true).
- `QUANT_DEFAULT_VISION_PROVIDER` — `off|manual|desktop_capture|direct_nexus|atlas_push_bridge|insta360` al iniciar API.
- `QUANT_CHART_PROVIDER` — `tradingview|yahoo` para selector y warmup.
- `QUANT_API_KEY` — header `X-API-Key` para readiness vía script.

---

## 4. Cómo reproducir verificación local

```powershell
cd C:\ATLAS_PUSH
# Reiniciar Quant si el binario no coincide con el repo (puerto 8795):
#   detener proceso en 8795, luego:
.\scripts\atlas_quant_start.ps1 -Port 8795 -MaxWaitSec 90

.\venv\Scripts\python.exe C:\ATLAS_PUSH\scripts\quant_reality_check.py --http http://127.0.0.1:8795
# Exit 0 esperado si readiness existe y ok.

.\venv\Scripts\python.exe -m pytest C:\ATLAS_PUSH\atlas_code_quant\tests\test_startup_visual_connect.py C:\ATLAS_PUSH\atlas_code_quant\tests\test_chart_execution_contract.py C:\ATLAS_PUSH\atlas_code_quant\tests\test_chart_execution_and_regime_hysteresis.py C:\ATLAS_PUSH\atlas_code_quant\tests\test_price_cycle_and_context_hysteresis.py -q
```

---

## 5. Riesgos y límites conocidos

- **Git push `variante/nueva`:** puede ser rechazado por GitHub si el historial de la rama contiene blobs >100MB (journal sqlite, logs). Mitigación previa: subir cambios vía rama limpia (`push/...`) con cherry-pick.
- **UI protegida:** no se modificaron `dashboard.html`, `workspace.html`, ni NEXUS dashboard salvo necesidad explícita del owner.
- **NEXUS :8002:** readiness reporta no alcanzable si el robot no está levantado; con `provider: off` el gate puede seguir coherente para pruebas sin robot.

---

## 6. Preguntas sugeridas para la IA auditora

1. ¿El contrato `ensure_chart_mission` (sin targets / sin auto-open / sin browser) está cubierto de forma suficiente?
2. ¿Duplicar readiness en v1 y v2 es aceptable o conviene deprecar una ruta?
3. ¿`SensorVisionService()` instanciado dentro de `_active_vision_provider()` en cada `proposal()` es coste aceptable frente a inyección de dependencias?
4. ¿Debe el warmup de charts en startup estar detrás de un flag adicional de “producción” para evitar abrir navegador en CI?

---

## 7. Estado al cerrar esta entrega

- **Servidor Quant :8795** reiniciado desde repo; **`quant_reality_check.py --http`** → **EXIT=0**, `http_readiness_ok: true`.
- **Pytest** focal: **20 passed** (incl. hysteresis/ciclos del commit anterior en el mismo run).

*Fin del informe.*
