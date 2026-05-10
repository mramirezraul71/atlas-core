# ATLAS T4 Robot — Documento de Arquitectura

## 1. Vista C4 — Contexto

Ver `docs/t4/diagrams/C4_context.mmd`.

Actores:
- **Operador humano** — interactúa exclusivamente con el dashboard.
- **Plus500 Futures Technologies (T4)** — sistema externo regulado (FCM CFTC/NFA) con WebSocket+Protobuf como API objetivo.
- **atlas-core** — cerebro multi-agente (`brain_core`, `autonomy_bridge`, `state_bus`, `journal`).
- **Doppler/Infisical** — secret manager SaaS, fuente de verdad de credenciales.
- **Grafana/Prometheus** — observabilidad serial.

## 2. Vista C4 — Contenedores

Ver `docs/t4/diagrams/C4_container.mmd`.

| Contenedor | Tecnología | Responsabilidad |
|---|---|---|
| `t4_robot` | Python 3.11 + asyncio + FastAPI | Bucle de trading T4 (data + execución + risk + audit) |
| `t4_dashboard` | Next.js 14 (App Router) + React + TS | UI operativa en tiempo real |
| `atlas-core` | Python (existente) | Cerebro cognitivo, autonomía, journal global |
| `redis` | Redis 7 (ya en docker-compose) | Bus pub/sub entre `t4_robot` y `atlas-core` |
| `postgres` | Postgres 16 (nuevo) | Journal estructurado de decisiones + audit |
| `prometheus` | Prometheus | Series temporales operativas |
| `grafana` | Grafana | Paneles agregados + alerting |
| `doppler-cli` | CLI Doppler | Inyección de secretos al entrypoint |

## 3. Decisiones arquitectónicas (ADR resumidas)

### ADR-001 — Servicio externo en lugar de módulo
**Decisión:** `t4_robot` corre como proceso separado, no como módulo `modules/humanoid/`.
**Razones:**
- Aislamiento de fallos (T4 reconexión no debe afectar al loop cognitivo).
- Despliegue independiente en VPS Chicago (cercanía CME).
- Lifecycle distinto: `t4_robot` corre 24/7 según calendario CME, `atlas-core` puede ciclar.
- Reuso completo del lenguaje `RuntimeMode`/`LiveSwitchState` a través de paquete compartido importable.

### ADR-002 — Paridad Paper/Live con una sola UI
**Decisión:** No existen rutas `/paper/*` y `/live/*` separadas. Existe una única sesión con `connection.mode ∈ {simulator, live}`.
**Razones:**
- Reducción de superficie de error humano (toggle único).
- Aplicación literal del criterio de éxito del briefing.
- Los componentes UI consumen el mismo schema; el badge `LIVE` se renderiza desde `LiveSwitchState`.

### ADR-003 — Redis pub/sub para el bus interno + ZMQ para cognitivo
**Decisión:** Redis ya está en `docker-compose.yml`. Lo usamos para eventos T4. ZeroMQ (ya en `requirements.txt`) sigue para el bus cognitivo de `modules/humanoid/`.
**Razones:** evitar reescribir capas existentes; aislamiento de canales.

### ADR-004 — Doppler como única fuente de secretos
**Decisión:** `.env` solo guarda `DOPPLER_TOKEN`. Todo lo demás se inyecta vía `doppler run -- python ...` o SDK Python.
**Razones:** instrucción explícita del briefing; rotación; audit log integrado en Doppler.

### ADR-005 — Dual-WS contra T4 (Data + Trading)
**Decisión:** Dos conexiones WSS simultáneas: una read-only para data feed (L1/L2/MBO/trades), otra autenticada para órdenes/cuenta.
**Razones:** anexo §8.2; aislamiento de back-pressure; reconexión independiente.

### ADR-006 — Protobuf vendored y versionado
**Decisión:** los `.proto` se versan en `atlas_code_quant/execution/brokers/t4/proto/` y se generan en build (no en runtime).
**Razones:** trazabilidad; reproducibilidad de builds; gating en CI ante cambios de protocolo.

### ADR-007 — Sizing siempre por equity vivo de la cuenta activa
**Decisión:** ni paper ni live operan con capital hardcoded. `t4_risk.py` lee `equity`, `available_margin`, `initial_margin` de la cuenta activa antes de cada decisión.
**Razones:** instrucción explícita del briefing ("adaptación al capital existente").

### ADR-008 — Confirmación humana antes de SUPERVISED_LIVE
**Decisión:** Se reutiliza `mode_switcher.toggle_paper_live()` (movimiento circular del ratón + TTS) **y además** se requiere `live_unlock_approved=true` en Doppler (`live` config). Doble factor.
**Razones:** seguridad operacional; trazabilidad jurídica.

## 4. Componentes del servicio `t4_robot`

```
                ┌─────────────────────────────────────────────┐
                │           Orchestrator (asyncio)             │
                │   Scanner → Signal → Risk → Execute → Audit  │
                └──┬────────┬────────┬────────┬────────────┬───┘
                   │        │        │        │            │
              ┌────▼──┐ ┌──▼────┐ ┌─▼────┐ ┌─▼──────┐ ┌──▼──────┐
              │Scanner│ │Signal │ │Risk  │ │T4 Adap.│ │Audit/Jrnl│
              │       │ │XGB+   │ │Engine│ │(Data+  │ │JSONL +   │
              │MES,   │ │LSTM+  │ │      │ │ Trade) │ │Postgres  │
              │MNQ,…  │ │HMM    │ │      │ │        │ │          │
              └───────┘ └───────┘ └──────┘ └────────┘ └──────────┘
                   ▲        ▲        ▲        ▲            ▲
                   └────────┴────────┴────────┴────────────┘
                              Redis pub/sub bus
                                     │
                              ┌──────┴───────┐
                              │ api_server   │  ← FastAPI WS/SSE/REST
                              │ bridge_client│  ← publica a atlas-core
                              └──────────────┘
```

## 5. Vista de secuencia (orden simple en live)

Ver `docs/t4/diagrams/pipeline_sequence.mmd`.

```
Dashboard          api_server       Orchestrator       Risk        T4Adapter        T4
   │                  │                  │             │              │             │
   │ POST /order      │                  │             │              │             │
   │─────────────────▶│                  │             │              │             │
   │                  │ propose          │             │              │             │
   │                  │─────────────────▶│             │              │             │
   │                  │                  │ validate    │              │             │
   │                  │                  │────────────▶│              │             │
   │                  │                  │   ok        │              │             │
   │                  │                  │◀────────────│              │             │
   │                  │                  │ place_order │              │             │
   │                  │                  │──────────── ─ ─ ─ ─ ─ ─ ─▶│ Protobuf    │
   │                  │                  │             │              │────────────▶│
   │                  │                  │             │              │   ack       │
   │                  │                  │             │              │◀────────────│
   │ WS: order_ack    │                  │             │              │             │
   │◀────────────────────────────────────────────────────────────────│             │
   │                  │                  │             │              │  fill       │
   │ WS: fill         │                  │             │              │◀────────────│
   │◀─────────────────────────────────────────────────────────────────│             │
   │                  │                  │ post_fill_audit │          │             │
   │                  │                  │─────────────────────────────────────────▶│ Postgres
```

## 6. Vista de estados (live unlock)

Ver `docs/t4/diagrams/live_unlock_state.mmd`.

```
[PAPER_BASELINE] ──request_unlock──▶ [PAPER_AGGRESSIVE]
[PAPER_AGGRESSIVE] ──readiness_ok + human_unlock_approved──▶ [SUPERVISED_LIVE]
[SUPERVISED_LIVE] ──30d métricas ok──▶ [GUARDED_LIVE]
[GUARDED_LIVE] ──full_live_globally_locked=false (raro)──▶ [FULL_LIVE]
Cualquier estado ──kill_switch_active──▶ [PAPER_BASELINE]
```

Tabla literal en `runtime_mode.py:_ALLOWED_TRANSITIONS`. El robot **no introduce nuevas transiciones**, solo consume las existentes.

## 7. Vista de despliegue

```
              ┌─────────────────────────────────────┐
              │   VPS Chicago (CME proximity)        │
              │                                     │
              │  docker compose:                    │
              │    t4_robot         (Python)        │
              │    redis                            │
              │    postgres                         │
              │    prometheus                       │
              │    grafana                          │
              └──────────┬──────────────────────────┘
                         │ WSS Protobuf
                         ▼
              ┌─────────────────────────────────────┐
              │   Plus500 Futures Technologies (T4) │
              │   Simulator | Live FCM              │
              └─────────────────────────────────────┘

              ┌─────────────────────────────────────┐
              │   Workstation EU (operador)         │
              │                                     │
              │   navegador → t4_dashboard (Vercel  │
              │   o self-hosted Next.js detrás de   │
              │   Cloudflare Tunnel hacia VPS)      │
              └─────────────────────────────────────┘
```

El **bucle crítico** (decisión↔T4) vive en Chicago. La UI puede vivir donde sea: solo *observa* y emite comandos.

## 8. No-goals (qué NO hace este sistema)

1. **No** opera CFDs Plus500 (sin API documentada).
2. **No** sustituye a las apps oficiales de Plus500/T4 donde la licencia lo prohíba.
3. **No** introduce nuevas transiciones de `RuntimeMode`; reutiliza la tabla existente.
4. **No** lee secretos de `.env` directamente fuera de `DOPPLER_TOKEN`.
5. **No** comparte estado (DB, journal, audit) entre paper y live; **mismo schema, instancias distintas** identificadas por `account_scope`.
