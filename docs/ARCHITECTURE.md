# ATLAS – Sistema Operativo Cognitivo

Arquitectura de referencia: minimalista en diseño, maximalista en capacidades. Autónomo, modular, elegante, seguro, auditable, escalable.

## Principios de diseño

1. **Minimalismo estructural** – Solo lo necesario en cada capa.
2. **Modularidad limpia** – Módulos con interfaz clara, sin acoplamiento oculto.
3. **Todo evento auditable** – Audit Engine recibe acciones relevantes.
4. **Fallback en cada capa** – Sin fallo en cascada; degradación controlada.
5. **No dependencias automáticas** – `/deps/check` reporta; nunca instala por sí solo.
6. **Autonomía controlada** – Policy Engine + modos (plan_only / execute_controlled / execute_auto).
7. **Respuestas claras** – Resumen corto, resultado técnico, evidencia, archivos, log, siguientes pasos.

---

## Mapa: especificación ↔ código

| Componente | Especificación | Ubicación | Notas |
|------------|----------------|-----------|--------|
| **Kernel** | Núcleo, registro, eventos | `modules/humanoid/kernel/` | Registry, event_bus, health |
| **Policy Engine** | Permisos por actor/rol/módulo/acción | `modules/humanoid/policy/` | engine, config, defaults |
| **Audit Engine** | Log persistente de eventos | `modules/humanoid/audit/` | db, logger |
| **Metrics** | Contadores y latencias | `modules/humanoid/metrics/` | store, middleware |
| **Memory Engine** | SQLite persistente, threads, FTS, export | `modules/humanoid/memory_engine/` | db, store, recall, export |
| **Orchestrator** | Goal → Plan → Execute → Verify → Critic → Store → Report | `modules/humanoid/orchestrator/` | orchestrator, critic, memory |
| **Hybrid LLM Router** | FAST/CHAT/CODE/REASON/TOOLS | `modules/llm/` (router, service) | Ollama, rutas |
| **Scheduler** | SQLite jobs, lease, backoff, concurrency=1 | `modules/humanoid/scheduler/` | db, engine, locking, runner |
| **Update Engine** | Git fetch, staging, smoke, merge/rollback | `modules/humanoid/update/` | update_engine, updater, rollback |
| **Watchdog** | Latency, error rate, scheduler | `modules/humanoid/watchdog/` | engine, rules |
| **Self-Healing** | Reinicio de loops internos (nunca proceso completo sin aprobación) | `modules/humanoid/healing/` | engine, restart_scheduler |
| **Vision** | Imagen + OCR + LLM vision, fallbacks | `modules/humanoid/vision/` | analyzer |
| **Web Navigator** | Playwright: open, click, fill, extract, screenshot | `modules/humanoid/web/` | navigator, policy |
| **Script Factory** | PowerShell/Python con logs y errores, validar antes de ejecutar | `modules/humanoid/scripts/` | factory, validators |
| **App Scaffolder** | Generar apps (fastapi, pwa, etc.) | `modules/humanoid/scaffolder/` | generator, templates |
| **Voice** | STT/TTS; voz no ejecuta sin confirmación explícita | `modules/humanoid/voice/` | controller, stt, tts |
| **Dependency Inspector** | Reporte de deps; nunca instala | `modules/humanoid/deps_checker.py` + `GET/POST /deps/check` | check_all |

---

## 1) Memory Engine (prioridad crítica)

- **DB:** `C:\ATLAS_PUSH\logs\atlas_memory.sqlite` (configurable vía `ATLAS_MEMORY_DB_PATH`).
- **Soporta:** threads, tasks, plans, runs, artifacts, decisions, summaries, FTS, export snapshot.
- **Fallbacks:** embeddings fallan → solo FTS; FTS no disponible → LIKE; todo falla → histórico simple por orden.
- **Integración:** cada goal, step, resultado y error → Memory + Audit.

---

## 2) Orchestrator

- **Pipeline:** Goal → Decompose → Plan → Execute → Verify → Critic → Store → Report.
- **Modos:** `plan_only` | `execute_controlled` (un paso solo con approve vía API) | `execute` (= execute_auto según policy).
- **Capacidades:** construir apps, generar código, scripts, ejecutar comandos, validar resultados, replan si falla (critic + 1 reintento), memoria contextual.

---

## 3) Vision

- **Flujo:** LLM vision → OCR → metadata básica (fallbacks en ese orden).
- **Endpoint:** `POST /vision/analyze`.
- **Respuesta:** texto extraído, estructura/entidades, insights, acciones sugeridas.

---

## 4) Web Navigation

- Playwright; si no disponible → módulo disabled y `/deps/check` lo reporta. No instalación automática.

---

## 5) Script Factory

- Genera PowerShell y Python con estructura, logs y manejo de errores. Validación antes de ejecutar. Guardado en memoria y auditoría al generar.

---

## 6) Scheduler

- SQLite, lease locking, stale recovery, backoff, concurrency=1, persistente. Endpoints en API.

---

## 7) Windows Update (real)

- Git fetch, compare, staging, smoke tests, merge o rollback. **Nunca apply sin permiso:** `UPDATE_APPLY=false` por defecto; policy y/o `force` explícito para apply.

---

## 8) Watchdog + Self-Healing

- Monitor: latency, error rate, scheduler. Acciones: reiniciar loops internos (p. ej. scheduler), cambiar modelo, reducir concurrencia. **Nunca reiniciar el proceso/sistema completo sin aprobación explícita.**

---

## 9) Voice

- STT (whisper/faster-whisper si existe), TTS (piper/pyttsx3 si existe). **Modo seguro:** la voz no ejecuta comandos sin confirmación explícita (cualquier integración que traduzca voz → comando debe pedir confirmación).

---

## 10) Dependency Check

- `GET/POST /deps/check`: detecta playwright, whisper, tesseract, ollama, sqlite, vision deps. **Nunca instala automáticamente.**

---

## 11) Respuestas profesionales

Formato objetivo en endpoints clave:

- **resumen** – Una línea.
- **resultado_tecnico** – Datos estructurados.
- **evidencia** – IDs, rutas, conteos.
- **archivos_creados** – Lista cuando aplique.
- **log_interno** – Referencia o fragmento si relevante.
- **siguientes_pasos** – Sugerencia opcional.

Evitar verbosidad; estética minimalista; datos concretos.

---

## 12) Fallbacks multivía

| Si falla | Fallback |
|----------|----------|
| Vision LLM | OCR |
| OCR | metadata |
| LLM heavy | FAST |
| Scheduler | skip + audit |
| Web | disabled + deps report |
| Voice | text only |
| Update apply | rollback / no apply |

---

## Commits sugeridos

- `feat(memory-orchestrator): persistent memory + executive brain`
- `feat(vision-web): image analysis + navigator`
- `feat(ops-core): scheduler + update engine + watchdog + healing`
- `feat(interface): voice + scaffolder + script factory + deps check`
