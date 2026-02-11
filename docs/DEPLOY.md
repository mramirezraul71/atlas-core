# Deploy: Blue-Green local y Canary Ramp-Up

Documentación de despliegue, health extendido y canary para ATLAS.

---

## 1. Blue-Green local

### Concepto

- **Instancia ACTIVA**: puerto configurado como `ACTIVE_PORT` (ej. 8791).
- **Instancia STAGING**: puerto `STAGING_PORT` (ej. 8792).

Flujo al ejecutar **POST /deploy/bluegreen**:

1. Lanzar nueva instancia en `STAGING_PORT`.
2. Ejecutar smoke tests contra STAGING.
3. Healthcheck 3 veces contra STAGING.
4. Si todo OK: cambiar puerto activo (actualizar estado y reiniciar servicio); detener instancia antigua.
5. Si falla: mantener instancia activa actual; registrar rollback y pasar a modo single-port.

### Config (`atlas.env`)

| Variable | Descripción | Ejemplo |
|----------|-------------|---------|
| `DEPLOY_MODE` | `single` o `bluegreen` | `single` |
| `ACTIVE_PORT` | Puerto de la instancia activa | `8791` |
| `STAGING_PORT` | Puerto de la instancia en pruebas | `8792` |
| `AUTO_SWITCH_ON_HEALTH` | Si hacer switch automático cuando health OK | `true` |

### Estado persistente

- `logs/deploy_state.json`: `mode`, `active_port`, `staging_port`, `last_deploy_ts`, `last_switch_ts`.
- El **service launcher** (`tools/service_launcher.py`), si `DEPLOY_MODE=bluegreen`, lee `active_port` de este archivo.

### Endpoints

- **GET /deploy/status**: modo, puertos, health_score, version, channel, last_deploy, canary_stats, sugerencias de fallback.
- **POST /deploy/bluegreen**: ejecuta el flujo. Query **`dry_run=true`** para simular (lanza staging, smoke, health; no hace switch).

### Fallbacks

- Si smoke o health fallan → `mode` se pone a `single` y no se hace switch.
- Si el switch del servicio falla → se audita; el estado de puertos no se actualiza hasta un reintento exitoso.

---

## 2. Health extendido (GET /health)

Incluye:

- **LLM latency avg**: promedio de latencia LLM (métricas).
- **Scheduler running**: scheduler habilitado y DB accesible.
- **Memory writable**: memoria habilitada y DB escribible.
- **DB integrity**: `PRAGMA quick_check` en scheduler DB (omitido si scheduler deshabilitado).
- **Port active**: instancia respondiendo.
- **Uptime**: segundos desde arranque de la API.

**Score 0-100**: se calcula con los checks aplicables; los checks omitidos (módulo deshabilitado) no penalizan.

---

## 3. Canary Ramp-Up

### Config

| Variable | Descripción | Ejemplo |
|----------|-------------|---------|
| `CANARY_ENABLED` | Activar canary | `false` |
| `CANARY_PERCENTAGE` | Fracción de tráfico a versión canary (0–1) | `0.2` |
| `CANARY_FEATURES` | Features en canary (lista separada por comas) | `vision,web,optimizer` |

### Uso en código

- `use_canary_version(feature)` indica si esta llamada debe usar la versión canary del feature.
- `record_canary_call(feature, used_canary, ok, latency_ms)` registra la llamada para métricas comparativas.

### Endpoints

- **GET /deploy/status**: incluye `canary_stats` (llamadas 1h, error rate, latencias).
- **GET /deploy/canary/report?hours=24**: informe canary vs stable en las últimas N horas.

### Fallback

- Si la tasa de error canary supera el umbral (15 %) → se setea `canary_disabled_fallback` en el estado y `is_canary_enabled()` devuelve `False` hasta que se resetee manualmente el estado.

---

## 4. Métricas (GET /metrics)

Se exponen contadores y latencias, entre ellos:

- `deploy_switch_total`, `deploy_switch_success`, `deploy_switch_fail`: switches blue-green.
- `deploy_bluegreen_runs`: ejecuciones del flujo (incl. dry_run).
- `canary_requests`: llamadas registradas con `record_canary_call`.
- `canary_<feature>`: latencias por feature canary.

---

## 5. OpenAPI

Los endpoints de deploy y health están agrupados con los tags **Deploy** y **Health** en la documentación Swagger (`/docs`).
