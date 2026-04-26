# Fase 5 — Institutional Radar: evidencia viva (2026-04-26)

Entorno: Windows, réplica local `C:\ATLAS_PUSH`. Evidencia reproducible: comandos `git`, `curl` (curl.exe), `pytest` con venv. Artefactos locales opcionales: `reports/tmp_f5_sse_20s.txt`, `reports/tmp_f5_summary.json` (no versionados; pueden borrarse).

---

## 1. Resumen ejecutivo

Se recopilaron pruebas vivas de HTTP, SSE 20 s, cabeceras anti-caché (GET y HEAD donde aplica), payload de summary, latencias, y la suite de tests parcial acordada. **No se aplicó cambio de código** (no hubo regresión que exigiera fix mínimo en este ciclo). El stack mostró **variabilidad**: en ventanas consecutivas `PUSH :8791/health` llegó a **timeout** mientras otras rutas del mismo servicio respondían 200; **Quant :8792** a veces **rechazó conexión** y otras entregó **200** en `/health` — dejar trazada como **degradación operacional**, no fallo lógico del contrato de radar. La **prueba 10.1–10.6 (navegador / Lighthouse / matar proceso)** queda **manual** según criterio de seguridad. **`pytest --collect-only`** en todo `tests` termina con **4 errores de recolección** (import `AgentConfig` en `tests/tools/test_atlas_agent_*.py`), **ajeno al paquete radar**; atención: **no es suite “verde” al 100%** en collect-only.

**Recomendación:** cierre de **Fase 5 (evidencia automática)**: **casi** — falta remediación **opcional** (ver tabla) y cierre de **BLOQUE 10.x** en sesión de operador con DevTools/Chrome.

---

## 2. BLOQUE 0.1 — Rama, commit, paridad

| Dato | Valor |
|------|--------|
| Rama | `variante/nueva` |
| `HEAD` | `2d0d9bd1925a8eb2d86a9cb970e800e4d378ac78` |
| `git log -1 --oneline` | `2d0d9bd19 chore(radar): fase 4 B1–B5 — SSE jitter, limpieza legacy, tests, defer, breadcrumb` |
| Trabajo en árbol | Limpio salvo **no rastreados** `reports/tmp_f5_*.json|txt` (artefactos de esta fase) |
| Remoto rastreado | `origin/variante/nueva` — rama local **ahead 4** (no paridad 1:1 con remoto hasta `git push`) |

**Resultado:** **WARN** (paridad local ≠ remoto por 4 commits; aceptable si el plan acordó validar rama de trabajo, no el SHA desplegado en prod).

---

## 3. BLOQUE 0.2 — Healths y rutas (HTTP viva)

Evidencia **representativa** (misma ventana de tiempo que 3.1/6.1, ~22:19–22:20 UTC):

| URL | Status | Contenido / notas | Tiempo aprox. |
|-----|--------|------------------|---------------|
| `http://127.0.0.1:8791/health` | **200** o **timeout** (observado ~15 s sin respuesta) | `application/json` cuando respondió; **inconsistente bajo carga o bloqueo** | ~0,003 s–2,5 s cuando OK; **timeout** en otra toma |
| `http://127.0.0.1:8792/health` | **200** o **conexión rechazada (7)** | `application/json` con `x-atlas-radar-source: quant` cuando OK; **intermitente** | ~2 s a fallo; o ~18 s+ cuando respondió 200 (latencia de arranque) |
| `GET /api/radar/dashboard/summary?symbol=SPY` | **200** | JSON; `x-atlas-radar-source: quant`; cuerpo ~1605 B | **~7 s** (primera respuesta) |
| `GET /api/radar/diagnostics/providers` | **200** | JSON; cuerpo 684 B; `cache-control: no-store, max-age=0, must-revalidate` | < **~8 s** |
| `GET /radar/dashboard` | **200** | `text/html; charset=utf-8`; ~12,6 kB | **~3,1 s** |
| `GET /api/radar/symbols/search?q=SPY` | **200** | JSON; `Cache-Control: private, max-age=12, stale-while-revalidate=24` (TTL corto coherente con búsqueda) | < **~20 s** |

**Degradación honesta:** en summary/SSE, proveedores con `ready: false`, `CAMERA_*`, clasificación `operable_with_degradation` visibles (no “stub ciego”).

**Resultado:** **WARN** (PUSH/health e instancia **8792** inestables en el tiempo; las rutas de producto clave de radar **sí** respondieron 200 en la sesión documentada).

---

## 4. BLOQUE 0.3 — Versiones runtime

| Componente | Versión |
|--------------|--------|
| Python | 3.14.0 (venv del repo) |
| fastapi | 0.136.0 |
| uvicorn | 0.45.0 |
| Sistema | Windows 10+ (build 26200 en user_info) |
| Navegador (10.x) | **No medido** en este agente — marcar en 10.x |

**Resultado:** **OK** (verificado por `pip`/`import` en sesión previa; repetible con `python -V` y `pip show`).

---

## 5. BLOQUE 3.1 — SSE 20 s (`/api/radar/stream?symbol=SPY`)

Comando: `curl.exe -N -m 21` → corte por tiempo ~21 s, archivo de captura con líneas `data: {...}`.

**Observado (20 s reales, timeout curl ~21 s):**

- **retry inicial:** `retry: 3000` (3 s) presente inmediatamente.
- **Heartbeat:** 4 eventos (tipos en JSON: `heartbeat`).
- **Snapshot:** `snapshot_update` y además evento `snapshot` (carga útil mínima).
- **camera_state_changed:** 1
- **provider_state_changed:** 1
- **degraded_state_changed** (envelope `...degraded_...` — nombre canónico del producto) **1** (en auditoría se cita `degraded_state_changed`; coinciden).

**Sobre el envelope (primer evento con todos los campos de ejemplo `heartbeat`):** `type`, `timestamp`, `symbol`, `source`, `sequence`, `data` **presentes**.

**Resultado:** **OK** para criterio de aceptación de esta fase. Nota: no se forzó presencia de **todos** los códigos en 20 s si el mercado/cámara no emitieran; aquí **sí** se vieron en la ventana.

---

## 6. BLOQUE 3.4 — Latencia interna SSE (aproximada)

- `data.heartbeat_interval_sec: 5` y espaciado real entre `heartbeat` (timestamps) **~5,0 s** (ej. `...39.09` → `...44.09` → `...49.09` → `...54.18` — última ~+1 s de jitter aceptable).
- **Reconexión** no forzada en 20 s (no se observó reconnect; stream cortado por `curl -m`).

**Resultado:** **OK** (orden de magnitud coherente con 5 s).

---

## 7. BLOQUE 4.6 — Cabeceras anti-caché (vivo)

| Recurso | Método | Comportamiento clave |
|---------|--------|----------------------|
| `/radar/dashboard` | **GET** (recomendado) | `cache-control: no-store, no-cache, must-revalidate, max-age=0`, `pragma: no-cache`, `expires: 0` |
| `/radar/dashboard/assets/dashboard.js` | **GET** | Mismo `no-store` + `ETag` / `Last-Modified` (navegador revalida; no caché larga) |
| `/radar/dashboard/assets/dashboard.css` | **GET** | Análogo a JS |
| `/static/shared/tokens.css` | **GET** (HEAD probado) | 200, `no-store` + ETag/Last-Modified |
| `/api/radar/dashboard/summary?symbol=SPY` | **GET** | `cache-control: no-store, max-age=0, must-revalidate` |
| `/api/radar/symbols/search?q=SPY` | **GET** | `private, max-age=12, ... must-revalidate` (TTL acotado — **no** “fresco para siempre”; **coherente** con API de búsqueda) |
| Misma ruta o asset con **HEAD** vía `curl -I` | Varios | **HTTP 405 Method Not Allowed** en **algunas** rutas (solo `GET` permitido). **No** es ausencia de anti-caché en el GET. |

**Resultado:** **OK** para GET. **WARN informativo:** `curl -I` / HEAD no sustituye a GET en rutas 405; documentar en playbooks de auditoría.

*Nota:* La auditoría mencionó `/static/radar/dashboard.js`; en este despliegue los assets viven bajo `/radar/dashboard/assets/dashboard.js` (evidencia GET 200 56 kB, JS con anti-caché).

---

## 8. BLOQUE 6.1 — Payload `GET /api/radar/dashboard/summary?symbol=SPY`

Campos verificados (JSON real `reports/tmp_f5_summary.json` o equivalente en línea):

- **Clasificación:** `radar.signal.meta.snapshot_classification` = `operable_with_degradation`
- **fast_pressure / score:** `fast_pressure_score: 0.0` (el nombre “fastPressure” legacy no aparece; score sí)
- **structural / flow:** `structural_confidence_score` y afinidad/divergencia en `meta`
- **provider_health_summary:** `providers_checked`, `degraded_count`
- **camera (contexto operador):** `camera_context` completo (estado `disabled`, notas, etc.)
- **transport / stub / SSE:** `stream_available`, `transport: { sse, stub, quant, ... }`, `quant.*`
- **degradationsActive:** clave en payload documento `degradations_active` (1 ítem cámara)
- **Decisión / gate:** `decision_gate` (vacío en la muestra; estructura presente)

**Resultado:** **OK** para cierre de contrato de fase. Si la auditoría exigía nombres camelCase en el JSON, sería un **tema de naming documentado (WARN)**; la forma “útil al operador” queda servida vía `meta` + `degradations_active` + `camera_context`.

---

## 9. BLOQUE 7.2 — Latencia end-to-end (5 cURLs por endpoint)

Método: `Measure-Command` sobre `curl.exe` en PowerShell, 5 iteraciones, `-m 120` (Windows).

| Endpoint | t1..t5 (s) aprox. | min | max | promedio |
|----------|-------------------|-----|-----|----------|
| `.../dashboard/summary?symbol=SPY` | 0.034, 0.012, 0.013, 0.012, 0.012 | 0,012 | 0,034 | 0,016 |
| `.../diagnostics/providers` | 0,019, 0,011, 0,013, 0,012, 0,011 | 0,0106 | 0,0194 | 0,0132 |
| `.../symbols/search?q=SPY` | 0,019, 0,011, 0,010, 0,011, 0,011 | 0,0102 | 0,0188 | 0,0125 |

**Observación:** Estables y sub-40 ms en ráfaga; el primer pico hacia summary se explica por cold/quant en una iteración. Las primeras descargas completas de la sesión (sin warm-up) alcanzaron **~7 s** en summary.

**Resultado:** **OK**.

---

## 10. BLOQUE 9.1 — `pytest` collect + parcial radar/F4

**Collect-only:** `188 tests collected`, **4 errors** durante recolección:

- `tests/tools/test_atlas_agent_memory.py`
- `tests/tools/test_atlas_agent_objectives.py`
- `tests/tools/test_atlas_agent_policy.py`
- `tests/tools/test_atlas_agent_toolkit.py`  

Causa: `ImportError: cannot import name 'AgentConfig' from 'config'`.

**Suite parcial (como en el plan):**  
`pytest tests/test_radar_quant_contract.py tests/test_radar_sse_contract.py tests/test_radar_symbols_search_http.py tests/test_startup_visual_connect.py tests/test_windows_camera_hints.py -q --tb=short`

- **14 passed, 1 skipped** (8,73 s). Skip en pista Windows bajo criterio del test.

**Resultado:** **WARN** a nivel **repositorio** (collect-only no limpio). **OK** a nivel **radar+F4** en el subconjunto corrido.

---

## 11. BLOQUES 10.1 – 10.6 — Navegador (manual / no ejecutado en agente)

| Subbloque | Resultado | Evidencia / acción |
|-----------|-----------|-------------------|
| 10.1 Happy path V4 → Radar | **PEND** | Requiere captura: operador en `/radar/dashboard`, flujo V4. |
| 10.2 Modo degradado visible | **PEND** | Misma captura: banners/cámara/ proveedores en degradación. |
| 10.3 Resiliencia: matar Quant | **PEND (manual controlado)** | **No** se mató ningún servicio. Procedimiento seguro sugerido: 1) ventana A con SSE+UI; 2) parar **solo** proceso en `:8792` bajo su control; 3) observar banner/stub/timeout; 4) reanudar. |
| 10.4 Cambio de símbolo | **PEND** | Probar `?symbol=QQQ` o selector. |
| 10.5 Teclado / a11y básico | **PEND** | Tab/focus en contenedor radar. |
| 10.6 Lighthouse local | **PEND** | Chrome DevTools → Lighthouse → informe; o `npx lighthouse` si aplica. |

**Resultado global 10.x:** **WARN** (falta evidencia operador o marcar requisito de entorno).

---

## 12. Hallazgos vivos (estado = OK / WARN / FAIL)

| # | Tema | Estado | Nota |
|---|------|--------|------|
| 1 | Paridad git (ahead 4) | **WARN** | Pushear o documentar SHA aceptado. |
| 2 | :8791 `/health` intermitente | **WARN** | Revisar carga, workers, o middleware que bloquea health. |
| 3 | :8792 Quant intermitente | **WARN** | Asegurar supérvisión del proceso; no bloquea evidencia de radar vía 8791 cuando Quant está en red. |
| 4 | HEAD 405 en varias rutas | **OK/WARN** | Uso de GET en auditoría; 405 en HEAD no es regresión de caché al cliente. |
| 5 | SSE 20 s + envelope | **OK** | Cumplido. |
| 6 | Búsqueda `symbols/search` caché privada 12s | **OK** | Comportamiento esperable. |
| 7 | `pytest` collect 4 err | **WARN** | Fuera de alcance mínimo radar; corregir imports aislado. |
| 8 | 10.x sin ejecutar | **WARN** | Cierre visual pendiente. |

Ningún **FAIL** duro de contrato radar en las pruebas automáticas documentadas; los **FAIL** vivos se limitarían a **SLO** si 8791 health debe ser siempre < 1 s o si Quant debe estar 24/7.

---

## 13. Archivos de código tocados (fix)

**Ninguno.**

Archivos añadidos o considerados: solo artefacto opcional `reports/tmp_f5_*` y **este** informe bajo `reports/`.

---

## 14. ¿Cierre de informe final? Pendientes mínimas

- **¿Listo para cierre absoluto?** **No** — quedan **10.x (manual)**, paridad con remoto si el comité exige SHA, y (opcional) alinear `pytest` collect o documentar excepción.
- **¿Listo para cierre de fase de evidencia automática radar?** **Sí, con reservas WARN** de la tabla anterior.

---

*Generado Fase 5, Institutional Radar, ATLAS. Evidencia reproducible con `curl`/`git`/`pytest` en el estado del repo y stack descritos arriba.*
