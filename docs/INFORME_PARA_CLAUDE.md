# Informe para Claude – Estado ATLAS PUSH (post Fase 5)

**Fecha:** 2025-02-14  
**Branch:** intent-input-rename  
**Objetivo:** Contexto actualizado para que Claude siga trabajando en el repo con coherencia.

---

## 1. Qué está hecho y funcionando

### Fase 4 – Autonomía e innovación
- **Meta-learning (MAML):** APIs en `atlas_http_api.py`: `/api/meta-learning/generate-tasks`, `/api/meta-learning/train`, `/api/meta-learning/adapt`, `/api/meta-learning/stats`. Dependen de `autonomous.learning.meta_learning` (torch).
- **Razonamiento causal:** `/api/causal/create-domain`, `/api/causal/reason`, `/api/causal/explain`, `/api/causal/counterfactual`. Usan `brain.reasoning.causal_model.CausalReasoner` (networkx).
- **Self-programming (Fase 4.3):** Módulo en `brain/self_programming/`: `code_generator.py` (validación sintaxis y seguridad), `skill_sandbox.py` (SkillSandbox con Docker opcional, SkillValidator), `skill_optimizer.py` (SkillOptimizer). APIs: `/api/self-programming/validate`, `/api/self-programming/optimize`, `/api/self-programming/execute-sandbox`.

### Fase 5 – Producción y observabilidad
- **Tests E2E:** `tests/e2e/test_autonomous_cycle.py` y `tests/e2e/test_innovation_features.py`. Requieren PUSH en `http://127.0.0.1:8791`. Si un endpoint devuelve 404, el test hace skip (no falla). Ejecución: `pytest tests/e2e/ -v`.
- **CI/CD:** `.github/workflows/deploy.yml` (build/push en release publicado, deploy por SSH si hay `DEPLOY_HOST`). `docker-compose.yml` con servicios: push (8791), nexus (8000), robot (8002), redis. `Dockerfile` en raíz para PUSH; `nexus/atlas_nexus/Dockerfile` y `nexus/atlas_nexus_robot/Dockerfile` para Nexus y Robot.
- **Scripts:** `scripts/health_check.sh` (comprueba PUSH, Nexus, Robot), `scripts/backup.sh` (DBs, config, snapshots últimos 7 días).
- **Observabilidad:** `modules/observability/`: `metrics.py` (Prometheus: Counter, Histogram, Gauge, `MetricsCollector`), `structured_logging.py` (StructuredLogger JSON), `middleware.py` (registro de requests). En la API: middleware de observabilidad; `GET /metrics/prometheus` (texto Prometheus); `GET /api/observability/metrics` (resumen JSON); tarea en lifespan que llama a `update_system_metrics()` cada 10 s.

### Dependencias y estructura
- **requirements.txt:** Formato una línea por paquete. Incluye: fastapi, uvicorn, pydantic, python-dotenv, httpx, psutil, scikit-learn, sentence-transformers, faiss-cpu, torch, opencv, networkx, docker, prometheus_client.
- **Rutas de código:** La API se ejecuta con la raíz del repo en `sys.path` (BASE_DIR). Los imports de “brain” son `brain.self_programming.*` y `brain.reasoning.causal_model` (paquetes en la raíz del repo, no bajo `modules/`).

---

## 2. Comprobaciones realizadas

- Imports críticos: `modules.observability.metrics`, `brain.self_programming.skill_optimizer` / `skill_sandbox`, `atlas_adapter.atlas_http_api` → OK.
- Tests E2E: 3 passed, 7 skipped (skipped por 404 en el servidor que estaba en 8791; con servidor actual deberían pasar más).

---

## 3. Pendientes / notas para Claude

- **Auth:** No hay `/api/auth/login` ni JWT. Los E2E no usan token. Cuando se implemente auth, se puede reutilizar un fixture `auth_token` en los tests y proteger endpoints sensibles.
- **Embeddings (regla de workspace):** En `modules/humanoid/memory_engine/` sigue pendiente: búsqueda semántica con embeddings, almacenar embeddings al guardar planes/runs, integración con Ollama o lib ligera; fallback: `recall_by_similarity` → `recall_by_query`.
- **Nexus/Robot:** Tienen Dockerfiles; el robot usa `backend/` como contexto (COPY backend/ y uvicorn main:app). Nexus usa `python nexus.py --mode api`.
- **Métricas:** `/metrics` (JSON) es el snapshot del metrics store existente; `/metrics/prometheus` es el texto para Prometheus. No reemplazar el primero.

---

## 4. Cómo seguir

- Para probar todo el stack: `docker compose up --build` (o levantar PUSH con `python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791`) y luego `pytest tests/e2e/ -v`.
- Documentación de detalle: `docs/INFORME_FASE_5.md`.

**Repo actualizado; este informe sirve de handoff para Claude.**
