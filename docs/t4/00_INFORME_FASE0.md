# ATLAS T4 Robot — Informe Fase 0 (Auditoría + Plan)

> Documento entregable de la Fase 0 del proyecto.
> **No** se ha tocado código de producción ni integrado credenciales todavía.
> Repositorio: `mramirezraul71/atlas-core` · rama base `intent-input-rename` · rama trabajo `feat/t4-robot-phase0`.

---

## 1. Resumen ejecutivo

El anexo normativo prescribe un robot autónomo de futuros sobre Plus500 Futures Technologies (T4) con paridad estricta Paper/Live, controlado desde un único dashboard, con stack moderno (WebSocket + Protobuf, asyncio, micro-servicios, Doppler para secretos) y trazabilidad completa.

La auditoría del repo `atlas-core` (rama `intent-input-rename`) confirma que **no partimos de cero**: ATLAS ya contiene una capa quant madura (`atlas_code_quant/`) con scanner, risk, paper broker, circuit breaker, runtime mode V4, live switch resolver, broker router y un placeholder explícito para futuros (`_route_future_placeholder`, comentado como *"Fase 5 — broker real no conectado"*). La integración T4 es precisamente esa Fase 5.

**Decisión arquitectónica raíz (alineada con tu respuesta a las preguntas bloqueantes):**

| Pregunta | Decisión |
|---|---|
| Alcance regulatorio | Cuenta T4 US ya disponible → diseñamos contra WebSocket+Protobuf real, paper en T4 Simulator |
| Integración con atlas-core | **Servicio externo + bridge** (proceso separado, comunicación vía `autonomy_bridge` y Redis pub/sub) |
| Dashboard | **Híbrido**: SPA Next.js operativa (book, fills, depth/MBO, toggle conexión) + Grafana para series históricas y agregados |
| Secretos | **Doppler/Infisical** SaaS; `.env` solo como bootstrap del SDK |

**Criterio de éxito reafirmado:** un operador alterna paper↔live cambiando únicamente conexión (Simulator vs Live) en el dashboard, con las mismas pantallas, módulos, métricas, flujos y sizing derivado del equity de la cuenta activa.

---

## 2. Inventario de código reutilizable

Auditoría literal de la rama `intent-input-rename` (commit `6201bca`). Lo que reutilizamos vs. lo que añadimos:

### 2.1 Módulos a **reutilizar tal cual** (cero modificaciones lógicas)

| Componente | Ruta | Razón |
|---|---|---|
| `RuntimeMode` enum + transiciones | `atlas_code_quant/operations/runtime_mode.py` | Ya define `PAPER_BASELINE`, `SUPERVISED_LIVE`, `GUARDED_LIVE`, `FULL_LIVE` con tabla de transiciones permitidas. El robot T4 hereda este lenguaje. |
| `LiveSwitchState` resolver | `atlas_code_quant/operations/live_switch.py` | Resuelve `effective_live_enabled` con `blocking_reasons`. El toggle del dashboard se renderiza directamente desde este payload. |
| `CircuitBreaker` (F19) | `atlas_code_quant/risk/circuit_breaker.py` | Estados `closed/open/half_open` + cooldown. Aplicable al pipeline T4 sin cambios. |
| `mode_switcher.toggle_paper_live()` | `atlas_code_quant/execution/mode_switcher.py` | Confirmación física (movimiento circular del ratón) + TTS + audit log JSONL. Reutilizable como gate previo a `LIVE_UNLOCK`. |
| `kill_switch.py` | `atlas_code_quant/operations/kill_switch.py` | Endpoint y semántica ya definidos. |
| `brain_bridge.py` | `atlas_code_quant/operations/brain_bridge.py` | Puente atlas_code_quant ↔ brain_core. El robot T4 publica eventos por aquí. |
| `realtime_feed.py` (estructura) | `atlas_code_quant/data/realtime_feed.py` | Patrón `OHLCVBar` + `FeedStats` + reconexión con backoff. Lo extendemos para T4 (no se duplica). |
| `PaperOrderResult`, `PaperPosition`, `PaperAccountSummary` | `atlas_code_quant/paper/paper_broker.py` | Dataclasses del universo paper. El **mismo** modelo se usa para Simulator T4 (clave para la paridad UI). |
| `BrokerRouter` | `atlas_code_quant/execution/broker_router.py` | Punto de extensión: `_route_future_placeholder` se sustituye por `_route_t4()` cuando el adapter esté listo. |
| Esquemas API (`OrderRequest`, `SignalResponse`, etc.) | `atlas_code_quant/api/schemas.py` | Contratos Pydantic ya consumidos por `api/main.py`. |
| `humanoid/orchestrator/` | `modules/humanoid/orchestrator/` | Plan→Propose→Execute→Audit. El servicio T4 lo usa como protocolo de autorización para órdenes en LIVE. |
| `autonomous/` (telemetry, resilience, health_monitor) | `autonomous/` | Watchdog + telemetría ya integrados en API principal. |
| `production/grafana_dashboard.py`, `grafana_pro.py` | `atlas_code_quant/production/` | Templates Grafana ya escritos. Se extienden con paneles T4. |

### 2.2 Módulos a **extender** (sin romper API existente)

| Componente | Extensión |
|---|---|
| `BrokerRouter._route_future_placeholder` | Reemplazo por `_route_t4(order_req, account_scope)` que delega en el nuevo `t4_adapter`. |
| `RuntimeMode` semántica | Misma enum, pero `account_scope="live"` ahora se interpreta como T4 Live FCM (no Tradier). |
| `live_switch.resolve_live_switch_state` | Sin cambios; se alimenta con readiness del adapter T4. |
| `mode_switcher._validate_live_activation` | Añadir checks `T4_LIVE_TOKEN`, `T4_LIVE_ACCOUNT_ID`, `T4_LIVE_FIRM` desde Doppler (no `os.getenv` directo). |
| `paper_broker` | Sin cambios. Se mantiene como fallback offline; T4 Simulator es preferente en paper. |

### 2.3 Nuevos paquetes (creación)

```
atlas_code_quant/
  execution/
    brokers/
      t4/                     ← NUEVO paquete
        __init__.py
        adapter.py            (T4Adapter público, dual-WS, reconexión)
        ws_client.py          (asyncio WSS client + heartbeat)
        proto/
          __init__.py
          (vendored .proto + generated _pb2.py)
        codec.py              (encode/decode Protobuf ⇄ dataclasses internos)
        auth.py               (handshake API-key, rotación)
        sessions.py           (data WS read-only ↔ trading WS stateful)
        subscriptions.py      (L1/L2/MBO/trade feed sub mgmt)
        order_book.py         (book reconstruction L2 + MBO)
        order_manager.py      (place/modify/cancel, bracket OCO, trailing)
        account_sync.py       (margen, equity, posiciones, fills)
        instruments.py        (specs MES/MNQ/MCL/MGC + tick sizes)
        risk_hooks.py         (pre-trade fat-finger + price bands)
        replay.py             (paper offline desde feed grabado)
        simulator.py          (cliente T4 Simulator — mismo wire-protocol)
        events.py             (T4Event dataclass + Redis publisher)
        config.py             (perfiles Simulator / Live)
        README.md
  api/
    routers/
      t4_router.py            ← NUEVO: REST + WS/SSE endpoints
  data/
    t4_feed.py                ← NUEVO: adapter de RealtimeFeed sobre T4
  scanner/
    futures_t4_scanner.py     ← NUEVO: hereda OpportunityScanner, watchlist MES/MNQ/MCL/MGC
  risk/
    t4_risk.py                ← NUEVO: sizing por margen T4 + correlación cross-contract
```

```
services/
  t4_robot/                   ← NUEVO servicio externo (proceso aparte)
    docker/
      Dockerfile
      entrypoint.sh
    src/
      main.py                 (asyncio runner, supervisor)
      orchestrator.py         (Scanner → Signal → Risk → Execute)
      api_server.py           (FastAPI: REST + WS + SSE para dashboard)
      bridge_client.py        (publica al autonomy_bridge de atlas-core)
      redis_bus.py            (pub/sub eventos)
      health.py               (readiness + liveness)
    pyproject.toml
    requirements.txt
    README.md
```

```
dashboard/
  t4_dashboard/               ← NUEVO frontend Next.js
    app/
      (paper)/                ← rutas
      (live)/
      _shared/                ← UN SOLO set de pantallas; paper/live solo cambian conexión
    components/
      DepthLadder.tsx
      MBOPanel.tsx
      OrderTicket.tsx
      PositionGrid.tsx
      RiskHUD.tsx
      ConnectionToggle.tsx    ← el único control que diferencia paper/live
      KillSwitchButton.tsx
      EquityCurve.tsx
      AuditTimeline.tsx
    lib/
      ws_client.ts            (WS al backend, reconexión, snapshot+diff)
      sse_client.ts
      schema.ts               (mirror de Pydantic)
    package.json
    next.config.ts
    README.md
```

```
docs/
  t4/
    00_INFORME_FASE0.md            ← este archivo
    01_ARQUITECTURA.md
    02_CONTRATOS_DE_DATOS.md
    03_API_TIEMPO_REAL.md
    04_PARIDAD_PAPER_LIVE.md
    05_RIESGO_Y_SIZING.md
    06_TESTS_Y_QA.md
    diagrams/
      C4_context.mmd
      C4_container.mmd
      pipeline_sequence.mmd
      live_unlock_state.mmd
T4_ROBOT_BUILD.md                  ← guía operativa de build/run (raíz docs/)
```

### 2.4 Conflictos detectados y desviaciones documentadas

1. **`mode_switcher.py` valida `TRADIER_LIVE_TOKEN`** explícitamente. Esto romperá si lo extendemos para T4. → *Desviación*: introducimos un `LiveCredentialsResolver` que abstrae la fuente (Doppler), y el switcher pasa a llamarlo por broker (`tradier` | `t4`). Conserva firma pública.
2. **El anexo prescribe Redis Pub/Sub**, pero el repo ya usa **ZeroMQ** (`pyzmq>=25` en `requirements.txt`) en la capa cognitiva y Redis aparece solo en `docker-compose.yml`. → *Decisión*: Redis para el bus T4 (anexo) + ZMQ para diálogo con cognitiva (existente). Documentado.
3. **El anexo recomienda VPS Chicago** para colocation. El user opera desde EU. → *Desviación operativa*: el servicio `t4_robot` es desplegable en VPS Chicago (Docker image), atlas-core sigue en local/EU. La latencia EU↔Chicago solo afecta al *dashboard*, no al bucle de trading.
4. **El anexo asume `.env`** en varios bloques. Tu instrucción explícita es **Doppler/Infisical**. → Resuelto: `.env` solo guarda `DOPPLER_TOKEN` (bootstrap), el resto se inyecta en runtime por el SDK Doppler. `.env.example` se actualiza con sección T4 marcada *"se obtiene de Doppler"*.

---

## 3. Mapeo Anexo → módulos implementables

Cada sección del anexo se materializa en uno o más componentes del repo:

| Sección anexo | Módulo objetivo | Ubicación |
|---|---|---|
| §3.2 Data Feed Layer | `t4_adapter.ws_client` + `t4_feed.py` + `subscriptions` | `atlas_code_quant/execution/brokers/t4/`, `atlas_code_quant/data/` |
| §3.3 Scanner | `futures_t4_scanner.py` (hereda `OpportunityScanner`) | `atlas_code_quant/scanner/` |
| §3.4 Signal Engine | XGBoost + LSTM + Regime HMM en `atlas_code_quant/models/` (existente); wrapper `t4_signal.py` | `atlas_code_quant/signals/` |
| §3.5 Risk Manager | `t4_risk.py` + reutiliza `circuit_breaker`, `kelly_engine`, `kill_switch` | `atlas_code_quant/risk/` |
| §3.6 Execution Engine | `t4.order_manager` + `BrokerRouter._route_t4()` | `atlas_code_quant/execution/brokers/t4/` |
| §3.7 Monitor/Dashboard | `services/t4_robot/api_server.py` + `dashboard/t4_dashboard/` + paneles Grafana | repo raíz |
| §4 Algoritmos | Estrategias existentes en `atlas_code_quant/strategies/` (ma_cross, momentum, regime, options, rl_strategy) | sin cambios estructurales |
| §5 KPIs | `monitoring/` + `journal/` + paneles dashboard | sin cambios; añadir paneles |
| §6 Stack | `requirements.txt` extendido (sección T4) + `pyproject.toml` del servicio | repo + service |
| §7 Workflow Research→Backtest→Paper→Live | `backtest/` + `lean/` + `paper/` + `t4.simulator` + `live_switch` | sin cambios; conectar simulator |
| §8 Diseño ATLAS sobre T4 | Toda la Fase 1-6 (ver §5 abajo) | servicio externo + módulos |
| §9 Regulatorio | `governance/`, `compliance` checks en `live_authorization.py` | extender |

---

## 4. Arquitectura propuesta (vista C4 — alto nivel)

```
┌────────────────────────────────────────────────────────────────────────────┐
│                            OPERADOR (browser)                              │
│              Next.js Dashboard SPA  (paper ≡ live, solo cambia conexión)   │
└──────────────┬─────────────────────────────────────────────────────────────┘
               │   WSS (snapshots+diffs) + SSE (events) + REST (commands)
┌──────────────▼─────────────────────────────────────────────────────────────┐
│  services/t4_robot  (proceso externo, Docker, idealmente VPS Chicago)      │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  FastAPI api_server   (REST + WS + SSE)                              │  │
│  │  Orchestrator         (Scanner → Signal → Risk → Execute)            │  │
│  │  T4 Adapter           (Dual-WS  Data / Trading, Protobuf, asyncio)   │  │
│  │  Order Book + MBO     (reconstrucción en memoria)                    │  │
│  │  Risk Engine          (sizing por equity/margen + circuit breakers)  │  │
│  │  Audit / Journal      (JSONL + SQLite + Postgres opcional)           │  │
│  └────────────┬──────────────────┬───────────────────┬───────────────────┘  │
└───────────────┼──────────────────┼───────────────────┼──────────────────────┘
                │ Redis pub/sub    │ WSS Protobuf      │ HTTPS (Doppler API)
                ▼                  ▼                   ▼
┌──────────────────────┐  ┌────────────────────┐  ┌──────────────────────┐
│  atlas-core          │  │  Plus500 T4        │  │  Doppler / Infisical │
│  brain_core /        │  │  WebSocket+Protobuf│  │  (secret manager)    │
│  autonomy_bridge /   │  │  Simulator | Live  │  └──────────────────────┘
│  state_bus / journal │  └────────────────────┘
└──────────────────────┘                ▲
                                        │ Grafana scraping
                                        ▼
                              ┌───────────────────────┐
                              │  Prometheus + Grafana │
                              │  (series + alertas)   │
                              └───────────────────────┘
```

Diagramas C4 y de secuencia en `docs/t4/diagrams/` (Mermaid).

---

## 5. Plan por fases (entregable principal)

Cada fase produce un PR auto-contenido contra `intent-input-rename`. **Nada se mergea sin tests verdes y revisión.** Las credenciales reales entran **solo a partir de la Fase 3**.

### Fase 0 — Auditoría + plan (este informe) — *ACTUAL*
**Entregables ya producidos en esta sesión:**
- `docs/t4/00_INFORME_FASE0.md` (este archivo)
- `docs/t4/01_ARQUITECTURA.md` (vistas C4 + decisiones)
- `docs/t4/02_CONTRATOS_DE_DATOS.md`
- `docs/t4/03_API_TIEMPO_REAL.md`
- `docs/t4/04_PARIDAD_PAPER_LIVE.md`
- `docs/t4/05_RIESGO_Y_SIZING.md`
- `docs/t4/06_TESTS_Y_QA.md`
- `docs/T4_ROBOT_BUILD.md` (build/run, env vars, orden de arranque, Doppler)
- Diagramas Mermaid en `docs/t4/diagrams/`

**Salida**: aprobación humana antes de Fase 1.

### Fase 1 — Esqueletos y contratos (sin red)
- Crear paquete `atlas_code_quant/execution/brokers/t4/` con stubs (sin lógica de WS).
- Vendar `.proto` de T4 (cuando estén disponibles); generar `_pb2.py` con `protoc`.
- Definir `T4Event` dataclasses y registrarlos en el bus interno.
- Crear servicio `services/t4_robot/` con FastAPI levantando `/health`, `/version`, `/status` (sin lógica de trading).
- Crear scaffolding del dashboard Next.js, vacío excepto `ConnectionToggle` y placeholder de pantallas.
- Tests unitarios de contratos (no red).

**Criterio salida**: `docker compose up t4_robot dashboard` levanta servicios y el dashboard muestra "Disconnected".

### Fase 2 — Paper local (sin T4 todavía)
- Conectar `paper_broker.py` existente al servicio `t4_robot` vía el adapter `simulator.py` en modo **local mock** (datos sintéticos OHLCV + book sintético).
- Implementar `OrderBook` en memoria y `MBOPanel` con datos mock.
- Dashboard renderiza book, fills, posiciones, equity y riesgo con datos del mock.
- Tests de paridad de payloads (mismo shape que producirá T4).

**Criterio salida**: el dashboard muestra una sesión paper completa sin diferencias estructurales respecto a lo que mostrará en live.

### Fase 3 — T4 Simulator (primera conexión real)
- Conectar al **T4 Simulator** real con credenciales de simulator (cuenta gratuita de demo a través del clearing).
- Reemplazar el mock por `T4Adapter` real en modo `account_scope=paper`.
- Validar reconexión, suscripciones L1/L2/MBO, fills sintéticos del simulator.
- Implementar `account_sync.py` (equity, margen, posiciones desde T4).
- Tests de integración contra el simulator (CI con credenciales de demo bloqueadas a sandbox).

**Criterio salida**: 72 h continuas paper en T4 Simulator sin pérdida de feed; reconciliación exacta posiciones/PnL adapter↔simulator.

### Fase 4 — Risk + paridad estricta
- Implementar `t4_risk.py` (sizing por equity y margen real de la cuenta activa).
- Cablear `circuit_breaker`, `kill_switch`, `live_authorization`, `live_guardrails` al pipeline.
- Tests de paridad: las **mismas** órdenes en paper y en live (modo dry-run) producen los **mismos** payloads de UI.
- Tests de "no efectos cruzados": ningún evento de live llega al journal de paper y viceversa.

**Criterio salida**: suite de tests de paridad y aislamiento pasa al 100%; informe automático paridad publicado en `reports/`.

### Fase 5 — Live FCM (cuenta real, capital mínimo)
- Subir credenciales **live** a Doppler (proyecto `atlas-t4`, env `live`).
- Habilitar `RuntimeMode.SUPERVISED_LIVE` con `require_human_unlock=true` y `full_live_globally_locked=true`.
- Primera operativa con **MES 1 contrato** y `daily_loss_limit = 0.5%` del equity.
- Audit log obligatorio en cada decisión + Telegram alerts.
- Watchdog supervisor con kill switch automático ante:
  - desconexión > 5 s,
  - drawdown > 1 % intradía,
  - latencia decisión→envío > 250 ms.

**Criterio salida**: 30 días en SUPERVISED_LIVE con métricas dentro de banda definida.

### Fase 6 — Escalado y promoción
- Promoción a `GUARDED_LIVE` solo tras validar Sharpe > 1.0, MDD < 5 %, fills > 95 % en target price.
- Ampliación de watchlist (MNQ, MCL, MGC micros).
- Activación de paneles Grafana avanzados + alertas Telegram por SLO breach.
- `FULL_LIVE` permanece **globalmente bloqueado** salvo decisión explícita; el resolver actual ya lo enforces.

**Criterio salida**: el operador alterna paper/live solo desde el toggle del dashboard, sin intervención técnica.

---

## 6. Tabla de variables de entorno (vista resumida)

> **Fuente única**: Doppler proyecto `atlas-t4`, configs `dev`, `paper`, `live`.
> `.env` local guarda **solo** `DOPPLER_TOKEN`. Detalle completo en `docs/T4_ROBOT_BUILD.md`.

| Variable | Scope | Origen | Notas |
|---|---|---|---|
| `DOPPLER_TOKEN` | dev/paper/live | `.env` local | Único secreto fuera de Doppler |
| `T4_API_HOST` | paper/live | Doppler | host WSS según entorno |
| `T4_API_KEY` | paper/live | Doppler | rotada |
| `T4_FIRM`, `T4_USER`, `T4_PASSWORD` | paper/live | Doppler | credenciales clearing |
| `T4_ACCOUNT_ID` | paper/live | Doppler | cuenta operativa |
| `T4_PROTO_VERSION` | dev | Doppler | versión .proto en uso |
| `ATLAS_T4_MODE` | runtime | env injection | `simulator` \| `live` |
| `REDIS_URL` | dev | Doppler | bus interno |
| `POSTGRES_URL` | dev | Doppler | journal + audit |
| `GRAFANA_API_TOKEN` | dev | Doppler | provisioning paneles |
| `TELEGRAM_BOT_TOKEN`, `TELEGRAM_CHAT_ID` | runtime | Doppler | alertas |
| `ATLAS_BRIDGE_URL`, `ATLAS_BRIDGE_TOKEN` | runtime | Doppler | conexión a atlas-core |

---

## 7. Riesgos y mitigaciones específicos

| Riesgo | Probabilidad | Impacto | Mitigación |
|---|---|---|---|
| T4 cambia versión de `.proto` | media | alto | versionado del paquete proto + smoke test en CI contra simulator |
| Reconexión T4 con posiciones abiertas | media | crítico | `account_sync` reconciliación obligatoria + circuit breaker abre el pipeline mientras dura la reconciliación |
| Doppler caído al arrancar | baja | medio | bootstrap con caché cifrado local de 24 h; el robot **no opera live** si la caché está caducada |
| Paridad UI rota tras refactor | media | alto | tests snapshot de payloads por componente |
| Slippage real >> backtest | alta | medio | factor de penalización en sizing + reporte semanal automático |
| Latencia EU↔Chicago al dashboard | alta | bajo | dashboard solo *observa*; el bucle de decisión vive en VPS Chicago |
| Live FCM exigencias regulatorias UE/US | alta | alto | gating jurídico fuera de scope técnico — flag `compliance_attested=true` requerido en Doppler para `GUARDED_LIVE` |

---

## 8. Próximos pasos inmediatos (post-aprobación)

1. Aprobar este informe y los documentos hermanos en `docs/t4/`.
2. Confirmar nombre del proyecto Doppler y crear las 3 configs (`dev`, `paper`, `live`).
3. Solicitar al clearing de T4 el paquete `.proto` y credenciales de simulator.
4. PR Fase 1 (esqueletos + scaffolding dashboard + `T4_ROBOT_BUILD.md` operacional).

Hasta que (1) esté aprobado, **no se toca código de producción ni se integran credenciales**. Esto cumple textualmente con tu instrucción de inicio.
