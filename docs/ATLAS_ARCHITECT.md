## ATLAS_ARCHITECT (ATLAS_PUSH)

Agente agentic de codificación orientado a **acciones reales**: indexa arquitectura, lee/escribe archivos con diff, ejecuta `pytest`/comandos permitidos y registra todo en Bitácora ANS.

### Qué resuelve (especificación)

- **Context-Aware Indexing**: genera `snapshots/architect/arch_index.json` con servicios/puertos y dependencias entre:
  - **PUSH**: `8791`
  - **NEXUS**: `8000`
  - **ROBOT**: `8002`
- **Filesystem Tool Use**: herramientas internas:
  - `read_file(path)`
  - `write_file(path, content, justification)` → crea diff en `logs/ans_architect_diffs/` y registra en ANS
  - `create_directory(path)`
  - `list_files(path=".", pattern="*", limit=200)`
- **Autonomous Terminal Execution**:
  - `run_pytest(nodeid=None)` y parse de fallos
  - `run_command(cmd, timeout_s=180)` (pasa por `SafeShellExecutor` y su allowlist)
- **Integration with ANS Logging**:
  - sugerencias y cambios quedan en `logs/ans_evolution_bitacora.json` (source=`atlas_architect`)
- **Multi-Model Orchestration**:
  - usa `modules.humanoid.ai.router.route_and_run` (free-first)
  - si se configura override manual, puede usar proveedor externo (p. ej. Anthropic con Sonnet) vía `brain_state`/policies.

### Código

- `modules/atlas_architect/indexer.py`: indexador de arquitectura (docker-compose + env)
- `modules/atlas_architect/fs_tools.py`: FS tools con diffs y logging
- `modules/atlas_architect/terminal_tools.py`: ejecución real + análisis de pytest
- `modules/atlas_architect/ans_logger.py`: Bitácora ANS (diff+justificación)
- `modules/atlas_architect/agent.py`: `AtlasArchitect` + loop agentic (tool_calls JSON)

### API (FastAPI)

Endpoints nuevos en `atlas_adapter/atlas_http_api.py`:

- `POST /api/architect/index`
- `POST /api/architect/pytest` body: `{ "nodeid": "tests/..." }`
- `POST /api/architect/propose-fix` body: `{ "problem": "...", "prefer_free": true, "context": {...} }`
- `POST /api/architect/agentic/execute` body: `{ "goal": "...", "prefer_free": true, "max_iters": 3 }`
- `POST /api/architect/order` body: `{ "order": "...", "mode": "governed|growth", "prefer_free": true, "max_heal_attempts": 3 }`
- `POST /api/architect/apply-approved` body: `{ "approval_id": "..." }`

### Gobernanza (GOVERNED vs GROWTH)

- **GOVERNED**:
  - genera un `plan` con `diff_preview` por archivo
  - encola una aprobación `action=architect_apply` en `/approvals/list`
  - no aplica cambios hasta que se apruebe y se ejecute `/api/architect/apply-approved`
- **GROWTH**:
  - aplica cambios (con diffs en `logs/ans_architect_diffs/`)
  - ejecuta tests y, si falla, activa `SelfHealingLoop`

