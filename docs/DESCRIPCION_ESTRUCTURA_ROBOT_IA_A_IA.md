# Descripción técnica estructura ATLAS (IA → IA)

**Uso:** Copiar y pegar a Claude Sonnet 4.5 (o modelo extendido) para handoff técnico. Lenguaje técnico IA-a-IA, sin simplificaciones.

---

## 1. Arquitectura de alto nivel

Sistema **cerebro–cuerpo–sensores** en monorepo **ATLAS_PUSH** (atlas-core):

- **PUSH (cerebro):** Orquestador central. API HTTP (FastAPI), dashboard unificado, memoria persistente, ANS (sistema nervioso autónomo), gobernanza (growth/governed/emergency), evolución (sandbox/validación/asimilación), cluster, gateway, aprobaciones owner.
- **NEXUS (cuerpo):** API de estado y ejecución. Neural Router (multi-LLM), Autonomous Engine (planning/execution), Tools Registry (web_search, web_scrape, file_read/write, system_command, pip_install, data_analysis, send_message), Directives Manager (inyección de contexto por proyecto), REST + WebSocket. Expone `/health`, `/status`, directivas, visión proxy.
- **Robot backend (sensores/efectores):** Cámaras, visión por computadora (YOLO, detección de objetos), flujo de video (MJPEG/stream), rutas de cerebro (AI router, reasoning, memoria a corto plazo), identidad (reconocimiento facial, voz). API REST + WebSocket en puerto distinto.

**Flujo de control:** PUSH mantiene heartbeat hacia NEXUS; NEXUS orquesta LLM + tools; Robot aporta percepción (visión) y puede recibir comandos. Dashboard (servido por PUSH) muestra estado agregado: PUSH, NEXUS, Robot.

---

## 2. Puertos y contratos

| Componente | Puerto | Entrypoint / API principal |
|-----------|--------|----------------------------|
| PUSH      | 8791   | `uvicorn atlas_adapter.atlas_http_api:app` — `/health`, `/status`, `/ui` (dashboard), `/api/nexus/*`, `/api/robot/status`, `/ans/*`, governance, evolution, product |
| NEXUS     | 8000   | `nexus/atlas_nexus/nexus.py --mode api` (uvicorn sobre `api/rest_api.py`) — `/health`, `/status`, `/directives/*`, `/api/*`, Neural Router, Autonomous Engine |
| Robot     | 8002   | `nexus/atlas_nexus_robot/backend/main.py` — `/`, `/api/camera/*`, `/api/vision/*`, `/api/brain/*`, YOLO, stream cámara |

Variables de entorno relevantes: `NEXUS_BASE_URL`, `NEXUS_ATLAS_PATH`, `NEXUS_ROBOT_PATH`, `NEXUS_TIMEOUT`; config en `config/atlas.env`.

---

## 3. PUSH (cerebro) — módulos clave

- **atlas_adapter/atlas_http_api.py:** FastAPI app, lifespan: registro de callbacks, arranque de heartbeat NEXUS, Prueba de Nervios (nexus_actions.run_nerve_test). Rutas: status, nexus connection/reconnect, robot status, cache/clear, ANS, GA, evolution, governance, cluster, product, owner.
- **modules/nexus_heartbeat.py:** Ping periódico a NEXUS (GET /health o /status), estado `nexus_connected`/`nexus_active`, auto-reactivación (restart_nexus: free port 8000, ejecución de start_nexus.ps1 o python nexus.py), callback a bitácora; `reconnect_nexus_and_poll()` para reconexión bajo demanda.
- **modules/nexus_client.py:** Cliente HTTP síncrono a NEXUS (get_nexus_status, etc.), timeout configurable.
- **modules/humanoid/ans/:** ANS (Autonomic Nervous System): engine (checks, heals, incidents), evolution_bitacora (deque en memoria, POST /ans/evolution-log), checks (api_health, deps_health, evolution_health, nexus_services_health), heals (install_optional_deps, restart_nexus_services, etc.), reporter, live_stream.
- **modules/humanoid/governance/:** Estado de gobernanza (growth | governed | emergency_stop) en SQLite + caché en memoria; API para set/get; notifier, audit.
- **modules/humanoid/ga/:** Governed Autonomy: executor, ciclos, reportes.
- **evolution_daemon.py / atlas_evolution.py:** Daemon de evolución (sandbox, tests, asimilación), integración con ANS y bitácora.
- **modules/brain_registry.py, modules/humanoid/:** Orchestrator, agents (executive, researcher), AI dynamic router, self_model, update_engine, plugin_expansion, auto_refactor, etc.
- **Dashboard:** `atlas_adapter/static/dashboard.html` — consume `/status`, `/api/nexus/reconnect`, `/api/cache/clear`, `/api/robot/status`; muestra estado NEXUS (8000) y Robot (8002); botones Reconectar NEXUS, Limpiar caché; iframe/stream para Robot.

---

## 4. NEXUS (cuerpo) — módulos clave

- **nexus/atlas_nexus/nexus.py:** Clase `AtlasNexus`: inicializa config (NexusConfig), NeuralRouter, ToolsRegistry, AutonomousEngine; `start_api_server()` monta uvicorn con `api/rest_api.create_api(self)`.
- **nexus/atlas_nexus/config/nexus_config.py:** PathConfig, OllamaConfig, DeepSeekConfig, BrainConfig (cache_responses, cache_ttl), ApiConfig (host/port), Environment.
- **nexus/atlas_nexus/brain/neural_router.py:** Enrutado multi-LLM por tipo de tarea (conversation, code, reasoning, etc.), TaskContext, TaskType, ResponseCache en memoria, integración Ollama/OpenAI/Anthropic.
- **nexus/atlas_nexus/brain/autonomous_engine.py:** Motor de planificación y ejecución autónoma (objetivos, uso de tools vía registry).
- **nexus/atlas_nexus/tools/tools_registry.py:** ToolsRegistry; herramientas: WebSearch, WebScrape, FileRead, FileWrite, SystemCommand, PipInstall, DataAnalysis, SendMessage; categorías (web, files, system, data, communication).
- **nexus/atlas_nexus/directives/:** DirectivesManager (estructura en disco, proyectos, directivas globales/por proyecto), directives_api (FastAPI router): resumen, health, toggle, templates; inyección de contexto para LLM.
- **nexus/atlas_nexus/api/rest_api.py:** create_api(nexus): rutas REST y posible WebSocket; registro de Directives API.

---

## 5. Robot backend (sensores/efectores)

- **nexus/atlas_nexus_robot/backend/main.py:** FastAPI, CORS; incluye vision_router, brain_router, camera_router; YOLO detector; modelos Pydantic para detección; WebSocket para stream/eventos.
- **api/camera_service_routes.py, api/vision_routes.py, api/brain_routes.py:** Rutas de cámaras (estado, stream), visión (detección), cerebro (router, reasoning, memoria a corto plazo).
- **vision/:** object_detection, cámaras (base, factory, detector, standard_webcam, network discoverer/scanner).
- **yolo_detector.py:** Detección de objetos con YOLO.
- **identity/:** face_recognition_system, voice_recognition_system.
- **brain/routing/:** query_classifier, ai_router; **brain/reasoning/:** personality_engine, logic_engine; **brain/memory/:** short_term.

---

## 6. Términos técnicos (glosario IA-a-IA)

- **Heartbeat:** Polling periódico (intervalo configurable) desde PUSH a NEXUS `/health` o `/status`; actualiza estado conectado/desconectado; tras N fallos consecutivos dispara restart_nexus (free port + start script).
- **Prueba de Nervios:** Validación física al inicio de PUSH: comprobación de canal con NEXUS/cuerpo (p. ej. mouse, cámara); resultado marca `nexus_active`.
- **Bitácora ANS:** Cola en memoria (evolution_bitacora) de eventos de evolución/conexión; POST /ans/evolution-log para append; dashboard/UI puede consumir para log visible.
- **Governance mode:** growth (ANS ejecuta heals sin aprobación) | governed (requiere aprobación) | emergency_stop (bloqueos); estado persistido en SQLite y caché.
- **Neural Router:** Componente NEXUS que selecciona modelo LLM según TaskType y contexto; cache de respuestas en memoria (TTL configurable).
- **Autonomous Engine:** Ejecutor de metas de alto nivel en NEXUS; planificación y llamadas a ToolsRegistry (tool use).
- **Directives:** Texto inyectado como contexto (global o por proyecto) para el LLM en NEXUS; gestionado por DirectivesManager; API para listar/actualizar/toggle.
- **Restart limpio:** Scripts `free_port.ps1` (genérico por puerto) y `restart_service_clean.ps1` (service=nexus|robot|push|all, clear cache: __pycache__, temp_models_cache); antes de start se libera puerto para evitar Errno 10048.
- **Dashboard unificado:** Una sola UI (PUSH :8791/ui); estado agregado cerebro + NEXUS (8000) + Robot (8002); Reconectar NEXUS (free port + clear_cache opcional + restart_nexus + poll); Limpiar caché (localStorage/sessionStorage + POST /api/cache/clear + refresh).

---

## 7. Rutas de archivo clave (raíz = ATLAS_PUSH)

```
atlas_adapter/atlas_http_api.py      # API PUSH, lifespan, rutas nexus/robot/ans/governance
atlas_adapter/static/dashboard.html  # UI unificada
modules/nexus_heartbeat.py           # ping_nexus, restart_nexus, clear_nexus_cache, reconnect_nexus_and_poll
modules/nexus_client.py              # Cliente HTTP NEXUS
modules/humanoid/ans/engine.py       # ANS engine, checks, heals
modules/humanoid/ans/evolution_bitacora.py
modules/humanoid/governance/state.py # Governance persistence
config/atlas.env                     # NEXUS_*, GOVERNANCE_*, ANS_*, etc.
nexus/atlas_nexus/nexus.py           # AtlasNexus, NeuralRouter, ToolsRegistry, AutonomousEngine
nexus/atlas_nexus/api/rest_api.py    # REST API NEXUS
nexus/atlas_nexus/brain/neural_router.py
nexus/atlas_nexus/brain/autonomous_engine.py
nexus/atlas_nexus/tools/tools_registry.py
nexus/atlas_nexus/directives/directives_manager.py
nexus/atlas_nexus_robot/backend/main.py # Robot FastAPI, vision + camera + brain routes
scripts/restart_service_clean.ps1    # Kill port + clear cache + start service
scripts/free_port.ps1                # Liberar puerto por número
```

---

*Documento para handoff técnico a Claude Sonnet 4.5 (extended). Repo: ATLAS_PUSH (atlas-core), monorepo cerebro + cuerpo + robot.*
