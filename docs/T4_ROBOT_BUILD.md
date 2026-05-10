# T4 Robot — Build & Run

Guía operativa para levantar el robot ATLAS T4 en local (desarrollo) y en VPS Chicago (producción). Vinculada a `docs/t4/00_INFORME_FASE0.md`.

> **Estado**: Fase 0 (auditoría + diseño). El servicio `services/t4_robot/` y el dashboard `dashboard/t4_dashboard/` aún **no** están creados. Esta guía describe el contrato que cumplirán en Fase 1.

---

## 1. Dependencias versionadas

### 1.1 Servicio Python (`services/t4_robot/requirements.txt`)
```
# Runtime
python>=3.11,<3.13
asyncio-mqtt==0.16.2                 # opcional para puente MQTT (off por defecto)
fastapi==0.115.0
uvicorn[standard]==0.30.6
websockets==13.0
httpx==0.27.2
pydantic==2.9.2
pydantic-settings==2.5.2

# T4 / Protobuf
protobuf==5.28.2
grpcio-tools==1.66.1                 # solo para `protoc` en build
betterproto==2.0.0b7                 # opcional, decode helpers

# Bus + datos
redis==5.0.8
psycopg[binary,pool]==3.2.2
sqlalchemy==2.0.35

# ML / análisis (reusados de atlas_code_quant)
pandas==2.2.3
numpy==1.26.4
ta-lib==0.4.32                       # binarios; en VPS usar wheel oficial
pandas-ta==0.3.14b0
scikit-learn==1.5.2
xgboost==2.1.1
lightgbm==4.5.0
hmmlearn==0.3.3
stable-baselines3==2.3.2
torch==2.4.1
# (CPU wheels para servicio; el cluster ML vive en otra máquina)

# Observabilidad
prometheus-client==0.20.0
structlog==24.4.0
sentry-sdk==2.13.0

# Secretos
doppler-cli                           # binario (instalado fuera de pip; ver §3)

# Auditoría
ulid-py==1.1.0
orjson==3.10.7

# Dev / Test
pytest==8.3.3
pytest-asyncio==0.24.0
pytest-cov==5.0.0
hypothesis==6.112.1
ruff==0.6.9
mypy==1.11.2
```

### 1.2 Dashboard (`dashboard/t4_dashboard/package.json` — extracto)
```jsonc
{
  "engines": { "node": ">=20.11" },
  "dependencies": {
    "next": "14.2.13",
    "react": "18.3.1",
    "react-dom": "18.3.1",
    "@tanstack/react-query": "5.56.2",
    "zustand": "4.5.5",
    "zod": "3.23.8",
    "klona": "2.0.6",
    "d3": "7.9.0",
    "recharts": "2.12.7",
    "framer-motion": "11.5.4",
    "tailwindcss": "3.4.13",
    "lucide-react": "0.445.0"
  },
  "devDependencies": {
    "typescript": "5.6.2",
    "vitest": "2.1.1",
    "@testing-library/react": "16.0.1",
    "@playwright/test": "1.47.2",
    "eslint": "9.11.1"
  }
}
```

---

## 2. protoc y `.proto` de T4

1. Solicitar al clearing de T4 el paquete oficial `.proto` (versión actual).
2. Copiar a `atlas_code_quant/execution/brokers/t4/proto/raw/` y **versionar** (sí, se commitean — sin secretos).
3. Generar Python:
   ```bash
   make -C atlas_code_quant/execution/brokers/t4 proto-gen
   ```
   El `Makefile` ejecuta:
   ```
   protoc -I=proto/raw \
     --python_out=proto/generated \
     --pyi_out=proto/generated \
     proto/raw/*.proto
   ```
4. Test golden round-trip se ejecuta automáticamente en CI: ver `docs/t4/06_TESTS_Y_QA.md §4`.

> **Política de cambio de protocolo**: si T4 publica nueva versión, se abre PR aparte que solo toca `proto/`, ejecuta golden tests y bumpea `T4_PROTO_VERSION`.

---

## 3. Doppler (gestión de secretos)

### 3.1 Setup local
```bash
# Una sola vez
brew install dopplerhq/cli/doppler            # macOS
# o
curl -Ls --tlsv1.2 --proto "=https" --retry 3 \
  https://cli.doppler.com/install.sh | sh     # Linux

doppler login
doppler setup --project atlas-t4 --config dev
```

### 3.2 Estructura de proyecto Doppler
```
atlas-t4/
  configs/
    dev      (desarrollo local, sin credenciales reales T4)
    paper    (T4 Simulator, credenciales del clearing)
    live     (T4 Live FCM, credenciales reales — ACL restrictiva)
```

### 3.3 Variables (vista canónica)
Solo `DOPPLER_TOKEN` vive en `.env` local. Todo lo demás se inyecta:

```bash
doppler run --config paper -- python -m services.t4_robot
```

Lista canónica (en cada config):
- `T4_API_HOST`, `T4_API_KEY`, `T4_FIRM`, `T4_USER`, `T4_PASSWORD`, `T4_ACCOUNT_ID`, `T4_PROTO_VERSION`
- `ATLAS_T4_MODE` (`simulator` | `live`)
- `REDIS_URL`, `POSTGRES_URL`
- `GRAFANA_API_TOKEN`, `PROMETHEUS_PUSHGATEWAY_URL` (opcional)
- `TELEGRAM_BOT_TOKEN`, `TELEGRAM_CHAT_ID`
- `ATLAS_BRIDGE_URL`, `ATLAS_BRIDGE_TOKEN`
- `T4_API_ALLOWED_CIDRS` (CSV)
- `LIVE_UNLOCK_APPROVED` (`true` solo en `live` y solo cuando el operador lo aprueba; se rota tras cada sesión)
- `COMPLIANCE_ATTESTED` (`true` para permitir `GUARDED_LIVE`; emisión manual con audit)
- `HARD_LIMITS_YAML_B64` (base64 del YAML de `HardLimits`)

### 3.4 Bootstrap caché cifrado (offline robust)
El servicio cachea los secretos cifrados con la clave derivada de `DOPPLER_TOKEN` durante 24 h en `~/.cache/atlas-t4/secrets.enc`. Si Doppler está caído al arrancar:
- en `dev`/`paper`: usa caché.
- en `live`: si la caché caducó > 24 h, el servicio se **niega a arrancar** y publica audit `live_blocked_no_secrets`.

---

## 4. Servicios y orden de arranque

### 4.1 docker compose (extracto)
```yaml
services:
  redis: { image: redis:7-alpine, ports: ["6379:6379"] }
  postgres:
    image: postgres:16-alpine
    environment: { POSTGRES_DB: atlas_t4, POSTGRES_USER: atlas, POSTGRES_PASSWORD_FILE: /run/secrets/pg_pw }
    secrets: [pg_pw]
    volumes: [pg-data:/var/lib/postgresql/data]
  prometheus:
    image: prom/prometheus:v2.54.1
    volumes: ["./infra/prometheus.yml:/etc/prometheus/prometheus.yml:ro"]
  grafana:
    image: grafana/grafana:11.2.0
    environment: { GF_SECURITY_ADMIN_PASSWORD_FILE: /run/secrets/gf_pw }
    volumes: ["./infra/grafana/provisioning:/etc/grafana/provisioning:ro"]
  t4_robot:
    build: ./services/t4_robot
    command: ["doppler","run","--config","${ATLAS_T4_DOPPLER_CONFIG:-paper}","--","python","-m","services.t4_robot"]
    environment:
      - DOPPLER_TOKEN=${DOPPLER_TOKEN}
    depends_on: [redis, postgres]
    ports: ["8801:8801"]
  t4_dashboard:
    build: ./dashboard/t4_dashboard
    environment:
      - NEXT_PUBLIC_T4_API_BASE=https://t4.atlas.local/t4/v1
    depends_on: [t4_robot]
    ports: ["3001:3000"]
secrets:
  pg_pw: { file: ./infra/secrets/pg_pw.txt }
  gf_pw: { file: ./infra/secrets/gf_pw.txt }
```

### 4.2 Orden de arranque canónico
1. `redis`, `postgres` (storage primero).
2. `prometheus`, `grafana` (observabilidad antes que workload).
3. `t4_robot` — espera healthchecks de redis y postgres; al boot:
   - resuelve `doppler run`,
   - corre migraciones `alembic upgrade head`,
   - publica `t4.audit` con `action=service_started`,
   - **no conecta a T4 todavía**: espera `POST /connection/connect`.
4. `t4_dashboard` — al cargar, llama `GET /t4/v1/status` y muestra `Disconnected`.
5. Operador presiona `ConnectionToggle` → `simulator` (o `live` con todos los gates pasados).

### 4.3 Atajo dev (sin docker)
```bash
# terminal 1
cd services/t4_robot
doppler run --config dev -- python -m services.t4_robot
# terminal 2
cd dashboard/t4_dashboard
pnpm dev
```

---

## 5. Tabla de variables de entorno (final)

| Nombre | Origen | Obligatoria | Notas |
|---|---|---|---|
| `DOPPLER_TOKEN` | `.env` local | sí | Único secreto fuera de Doppler |
| `ATLAS_T4_DOPPLER_CONFIG` | shell | sí | `dev`/`paper`/`live` |
| `T4_API_HOST` | Doppler | sí | host WSS |
| `T4_API_KEY` | Doppler | sí | rotada |
| `T4_FIRM` | Doppler | sí | clearing firm |
| `T4_USER` | Doppler | sí | usuario T4 |
| `T4_PASSWORD` | Doppler | sí | password T4 |
| `T4_ACCOUNT_ID` | Doppler | sí | cuenta operativa |
| `T4_PROTO_VERSION` | Doppler | sí | gating en CI |
| `ATLAS_T4_MODE` | Doppler | sí | `simulator`/`live` |
| `REDIS_URL` | Doppler | sí | bus interno |
| `POSTGRES_URL` | Doppler | sí | journal |
| `LIVE_UNLOCK_APPROVED` | Doppler (`live`) | solo live | rota por sesión |
| `COMPLIANCE_ATTESTED` | Doppler (`live`) | para `GUARDED_LIVE`+ | emisión manual |
| `HARD_LIMITS_YAML_B64` | Doppler | sí | `HardLimits` por config |
| `TELEGRAM_BOT_TOKEN` | Doppler | opc. | alertas |
| `TELEGRAM_CHAT_ID` | Doppler | opc. | alertas |
| `ATLAS_BRIDGE_URL` | Doppler | sí | conexión a atlas-core |
| `ATLAS_BRIDGE_TOKEN` | Doppler | sí | bearer del bridge |
| `T4_API_ALLOWED_CIDRS` | Doppler | opc. | IP allowlist |
| `SENTRY_DSN` | Doppler | opc. | telemetría errores |

---

## 6. Smoke checks (post-deploy)
```bash
# liveness
curl -fsS https://t4.atlas.local/t4/v1/health
# readiness
curl -fsS -H "Authorization: Bearer $JWT" https://t4.atlas.local/t4/v1/ready
# status (conexión, runtime mode, breaker state)
curl -fsS -H "Authorization: Bearer $JWT" https://t4.atlas.local/t4/v1/status | jq .
# WS feed
wscat -c "wss://t4.atlas.local/t4/v1/ws" -H "Authorization: Bearer $JWT"
> {"op":"subscribe","topics":["t4.feed.quote"]}
```

---

## 7. Política de cambios

- Toda modificación al servicio T4 entra por PR contra `intent-input-rename` (no a `main`).
- PRs que tocan `proto/` requieren label `t4-protocol-change` y revisión doble.
- PRs que tocan `live`-related code requieren label `live-impact` y firma de operador en el commit (`Signed-off-by`).
- Cualquier modificación de `HardLimits` requiere actualizar Doppler `live` **antes** del merge.

---

## 8. Referencias cruzadas
- `docs/t4/00_INFORME_FASE0.md` — informe completo y plan por fases
- `docs/t4/01_ARQUITECTURA.md` — vistas C4 + ADRs
- `docs/t4/02_CONTRATOS_DE_DATOS.md` — schemas de eventos y REST
- `docs/t4/03_API_TIEMPO_REAL.md` — endpoints + WS/SSE
- `docs/t4/04_PARIDAD_PAPER_LIVE.md` — enforcement de paridad
- `docs/t4/05_RIESGO_Y_SIZING.md` — fórmulas + circuit breakers
- `docs/t4/06_TESTS_Y_QA.md` — pirámide y obligatorios
- Código existente reutilizado:
  - `atlas_code_quant/operations/runtime_mode.py`
  - `atlas_code_quant/operations/live_switch.py`
  - `atlas_code_quant/risk/circuit_breaker.py`
  - `atlas_code_quant/execution/broker_router.py`
  - `atlas_code_quant/execution/mode_switcher.py`
  - `atlas_code_quant/paper/paper_broker.py`
  - `atlas_code_quant/operations/brain_bridge.py`
  - `atlas_code_quant/operations/kill_switch.py`
  - `atlas_code_quant/data/realtime_feed.py`
