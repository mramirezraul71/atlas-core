# Atlas Push · Estado actual del repo

> Snapshot del repositorio `mramirezraul71/atlas-core` (rama `main`)
> a fecha del Paso 1 de la reestructuración.
>
> Este documento describe qué hay **hoy**. No describe el objetivo
> (eso vive en `ARCHITECTURE.md`).

> **Nota post‑A2 (no reescribe este documento).**
> Este texto refleja el estado **anterior** al PR A2. Tras la fusión
> de A2, algunos de los archivos mencionados aquí por su ruta original
> han sido **movidos a `legacy/`** (ver `legacy/README.md`). En
> particular: `atlas.py`, `atlas_runtime.py`, `bridge/atlas_api_min.py`
> y `atlas_adapter/run_atlas_api.cmd`. Las rutas listadas más abajo
> se conservan intactas deliberadamente para no desvirtuar el
> snapshot histórico; la descripción del perímetro vivo posterior
> a A2 se mantendrá en `ARCHITECTURE.md` y, a partir del paso B, en
> un apartado dedicado de ese documento.

> **Nota post‑D (no reescribe este documento).**
> Tras la fusión del PR D, `atlas_push/` contiene tres paquetes
> nuevos además de `intents/`:
>
> - `atlas_push/engine/` con `DecisionEngine` (scaffold pass-through)
>   y los Protocols `Strategy` y `RiskManager`.
> - `atlas_push/state/` con `MarketState`, `AccountSnapshot`,
>   `Position`, `Quote`, `RiskContext`.
> - `atlas_push/outputs/` con `DecisionOutput`, `LogicalOrder`,
>   `TargetWeight`, `RiskVeto`.
>
> Todo con `@dataclass(frozen=True)`. `DecisionEngine.decide(state)`
> devuelve `DecisionOutput.empty()` para cualquier entrada; D no
> cablea el motor al `IntentRouter` ni al adapter HTTP. El contrato
> completo vive en `ARCHITECTURE.md` §3; el alcance, los tests y el
> protocolo de commits están en `PLAN_STEP_D.md`. Las rutas de la
> sección 4 no se reescriben; siguen reflejando el snapshot
> histórico previo a B.

## 1. Perímetro analizado

Incluido en el análisis (por decisión del owner):

- `atlas.py`
- `atlas_runtime.py`
- `core/` (`__init__.py`, `logger.py`, `scheduler.py`)
- `runtime/` (`start_atlas.ps1`)
- `modules/` (todos los `.py`)
- `bridge/` (`atlas_api_min.py`, `server.py`)
- `atlas_adapter/` (`atlas_http_api.py`, `run_atlas_api.cmd`)

Excluido del análisis:

- `rauli_core/`, `rauli_core_app/` (app Flutter, fuera del cerebro).
- `logs/`, `snapshots/`, `.venv/` (ruido de repo; PR de higiene
  aparte).
- Scripts `.ps1` de setup (tooling, no cerebro).
- `CURSOR_PROMPT.md`.

## 2. Resumen ejecutivo

- No existe hoy un "cerebro Atlas Push" propiamente dicho. Lo que hay
  es un **router de comandos textual** (`modules/command_router.py`)
  que resuelve `/status`, `/doctor`, `/notes`, `/snapshot` y un
  fallback a Inbox.
- Hay **cuatro APIs HTTP** paralelas envolviendo routers
  equivalentes; tres son legado.
- No existe en este repo una implementación explícita y estructurada
  de señales, riesgo, journal o decisiones de trading dentro del
  perímetro analizado de Atlas Push.
- El "cerebro" actual se reduce al router de comandos y su adapter
  HTTP. Todo lo demás del perímetro es interfaz, ops o legado.

## 3. Perímetro refinado de Atlas Push (aprobado)

Lo que se considera "Atlas Push real de hoy":

- `modules/command_router.py`
- `atlas_adapter/atlas_http_api.py`
- `core/logger.py`
- `core/__init__.py`
- `modules/__init__.py`

Todo lo demás del perímetro analizado queda fuera del cerebro para
esta reestructuración.

## 4. Clasificación de archivos

### 4.1 Esenciales (núcleo actual)

| Ruta | Rol | Nota |
|---|---|---|
| `modules/command_router.py` | Router `handle(text)` con comandos `/status`, `/doctor`, `/modules`, `/snapshot`, `/note`, NLP ES + fallback Inbox. | Semilla de `IntentRouter`. |
| `atlas_adapter/atlas_http_api.py` | FastAPI vigente (`/status`, `/tools`, `/execute`, `/modules`, `/intent`). | Entry-point preferido. Tiene deuda técnica (ver `DEFECTS.md`). |
| `core/logger.py` | `log(msg)` a `C:\ATLAS\logs\atlas.log`. | Usado transversalmente. |
| `core/__init__.py`, `modules/__init__.py` | Markers de paquete. | — |

### 4.2 Archivables (aprobados por el owner, se moverán en PR A2)

| Ruta | Motivo |
|---|---|
| `atlas.py` | REPL con TTS anterior al router. Nadie lo importa. |
| `atlas_runtime.py` | Router v1 sustituido por `command_router.py`. Solo lo mantiene vivo `bridge/atlas_api_min.py`. |
| `bridge/atlas_api_min.py` | API legada sobre router v1. |
| `atlas_adapter/run_atlas_api.cmd` | Script legacy con rutas `C:\ATLAS` obsoletas. Reemplazado por `03_run_atlas_api.ps1`. |

### 4.3 Fuera del perímetro de Atlas Push (interfaz / periferia / ops)

Por decisión del owner, estos archivos no forman parte del cerebro y
no se modifican en esta reestructuración:

| Ruta | Categoría | Nota |
|---|---|---|
| `core/scheduler.py` | Huérfano | Heartbeat no importado ni arrancado. |
| `modules/atlas_remote_api.py` | Interfaz legada | **Roto**: import inexistente (ver `DEFECTS.md`). Fuera de servicio. |
| `modules/atlas_telegram.py` | Canal de entrada | Telegram con NLP. |
| `modules/atlas_telegram_bot.py` | Canal de entrada legado | Versión pobre del anterior. |
| `modules/atlas_chat.py` | CLI Flutter | Invoca `flutter doctor/run`. |
| `modules/atlas_gui.py` | GUI Tk | Ventana Tkinter. |
| `modules/atlas_llm.py` | Wrapper OpenAI | Servicio auxiliar, no cerebro. |
| `modules/rauli_doctor.py` | Tooling ops | Verificación filesystem. |
| `modules/snapshot_engine.py` | Tooling ops | Backup de carpetas. |
| `bridge/server.py` | Ops/supervisión | Ejecuta shell con whitelist. Arranca desde `start_atlas.ps1`. |
| `runtime/start_atlas.ps1` | Ops | Lanza `bridge.server:app` + cloudflared. |

## 5. Grafo de dependencias actual (imports internos)

```
core/logger.py
    ▲
    ├── core/scheduler.py            (huérfano)
    ├── modules/snapshot_engine.py   ◄── atlas_chat, atlas_gui, atlas_telegram_bot
    ├── modules/atlas_chat.py
    ├── modules/atlas_gui.py         ──► rauli_doctor, snapshot_engine
    ├── modules/atlas_telegram_bot.py ──► rauli_doctor, snapshot_engine
    └── (sin uso directo desde command_router)

NÚCLEO ACTUAL
    modules/command_router.py  (handle)
        ▲                  ▲
        │                  │
    atlas_http_api.py   atlas_remote_api.py
    (adapter vigente)   (legado + roto)
                                │
                                │
                          atlas_llm.py ──► atlas_telegram.py

ROUTER LEGADO v1
    atlas_runtime.py
        ▲
        │
    bridge/atlas_api_min.py

OPS (fuera del cerebro)
    bridge/server.py  ◄── runtime/start_atlas.ps1
    atlas_adapter/run_atlas_api.cmd (legacy)

HUÉRFANOS
    atlas.py              (REPL pyttsx3)
    core/scheduler.py     (heartbeat)
```

## 6. Entry-points HTTP observados

`03_run_atlas_api.ps1` prueba 4 entry-points en cascada. Solo el
primero es el vigente:

1. `atlas_adapter.atlas_http_api:app` — **vigente**, usa
   `command_router`.
2. `bridge.server:app` — ops / ejecutor shell (fuera del cerebro).
3. `bridge.atlas_api_min:app` — legado (router v1).
4. `modules.atlas_remote_api:app` — legado + roto.

## 7. Qué falta por implementar (hoy no existe)

> Apartado escrito antes de B/C/D. Tras el merge de D, los tipos de
> estado y salida marcados con ✓ ya existen como scaffold en
> `atlas_push/` (ver nota post-D arriba y `PLAN_STEP_D.md`). Se
> conserva el listado original como referencia histórica.

- Cualquier concepto de `Strategy`, `Signal`, `Portfolio`. *(Protocol
  `Strategy` declarado en D; implementaciones reales siguen
  pendientes.)*
- Cualquier `RiskManager`, `RiskContext` o `limits`. *(Protocol
  `RiskManager` y `RiskContext` declarados en D; `limits.py` y la
  implementación real siguen pendientes.)*
- ✓ Cualquier `MarketState`, `Position`, `Quote`, `AccountSnapshot`.
  *(Declarados en D como `@dataclass(frozen=True)`.)*
- ✓ Cualquier `DecisionOutput`, `LogicalOrder`, `TargetWeight`.
  *(Declarados en D como `@dataclass(frozen=True)`, más `RiskVeto`.)*
- Cualquier puerto (`MarketStateProvider`, `ExecutionEngineAdapter`,
  `JournalSink`). *(Parte del paso E.)*
- Cualquier adapter LEAN. *(Parte del paso E o posterior.)*

Estos elementos se crean desde cero en los pasos D y E del plan de
refactor, no se migran.
