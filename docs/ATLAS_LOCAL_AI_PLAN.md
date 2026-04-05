# ATLAS Local AI Plan

## Goal

Build a local-first AI stack for ATLAS that:

- preserves the current repo structure,
- covers the whole system instead of a single module,
- keeps Telegram and Bitacora as mandatory observability channels,
- upgrades local model coverage without disrupting the current runtime.

## Current Reality

ATLAS already has:

- local Ollama models,
- a role-based model registry,
- Telegram notification plumbing,
- Bitacora and audit endpoints,
- a live event bus.

This means the correct path is not a structural rewrite. The correct path is a staged expansion of the existing local model layer.

## Existing Local Models

Verified via `ollama list`:

- `mistral-small:24b`
- `qwen3-vl:30b`
- `qwen3-coder:30b`
- `qwen3:30b`
- `nomic-embed-text:latest`
- `qwen2.5-coder:1.5b-base`
- `llama3.1:8b`
- `llava:7b`
- `deepseek-coder:6.7b`
- `qwen2.5:7b`
- `llama3.2:3b`
- `deepseek-coder:1.3b`
- `llama3.2-vision:11b`
- `codellama:latest`
- `llama2:latest`
- `deepseek-r1:latest`
- `llama3:latest`
- `llama3.1:latest`
- `deepseek-r1:14b`
- `qwen3:4b`

## Existing ATLAS Model Routing

Current role defaults in `modules/humanoid/ai/registry.py`:

- `FAST` -> `ollama:llama3.2:3b`
- `CHAT` -> `ollama:llama3.1:latest`
- `CODE` -> `ollama:deepseek-coder:6.7b`
- `REASON` -> `ollama:deepseek-r1:14b`
- `TOOLS` -> `ollama:qwen2.5:7b`
- `VISION` -> `ollama:llama3.2-vision:11b`
- `ARCHITECT` -> `ollama:deepseek-r1:14b`
- `OPTIMIZER` -> `ollama:deepseek-coder:6.7b`

Current brain state defaults in `modules/humanoid/ai/brain_state.py`:

- `code` -> `ollama:deepseek-coder:6.7b`
- `vision` -> `ollama:llama3.2-vision:11b`
- `chat` -> `ollama:llama3.1:latest`
- `analysis` -> `ollama:deepseek-r1:14b`

## Existing Observability Wiring

Telegram:

- stable notification path: `modules/humanoid/notify.py`
- bridge/poller support: `modules/humanoid/comms/telegram_bridge.py`, `modules/humanoid/comms/telegram_poller.py`
- supervisor already emits critical Telegram notifications

Bitacora:

- persistent journal: `modules/humanoid/ans/evolution_bitacora.py`
- write endpoint: `POST /bitacora/log`
- read stream: `GET /bitacora/stream`
- audit tail: `GET /audit/tail`

Verified runtime endpoints:

- `GET /health` -> `200`
- `GET /bitacora/stream` -> `200`
- `GET /api/comms/status` -> `200`
- `GET /api/kernel/event-bus/stats` -> `200`

## Coverage Gaps

The current local stack is solid as a baseline, but still limited for full-system coverage:

1. General reasoning is usable but not yet strong enough for repo-scale orchestration.
2. Vision is present, but there is no stronger long-context multimodal model for UI, dashboard, OCR, and camera reasoning at higher fidelity.
3. Code reasoning exists, but there is no large local code-specialist for repository-scale understanding.
4. The stack is missing a stronger local tool-using generalist.

## Target Stack

### Keep

- `llama3.2:3b` for `FAST`
- `deepseek-r1:14b` as fallback `REASON`
- `deepseek-coder:6.7b` as fallback `CODE`
- `nomic-embed-text:latest` for embeddings
- `llama3.2-vision:11b` as fallback vision model

### Add

- `qwen3:30b`
- `qwen3-coder:30b`
- `qwen3-vl:30b`
- `mistral-small:24b`

## Recommended Role Mapping

### Conservative Upgrade

Use this if local resources are limited:

- `FAST` -> `ollama:llama3.2:3b`
- `CHAT` -> `ollama:qwen3:30b`
- `CODE` -> `ollama:qwen3-coder:30b`
- `REASON` -> `ollama:deepseek-r1:14b`
- `TOOLS` -> `ollama:qwen3:30b`
- `VISION` -> `ollama:qwen3-vl:30b`
- `ARCHITECT` -> `ollama:mistral-small:24b`
- `OPTIMIZER` -> `ollama:qwen3-coder:30b`

### Stronger Upgrade

Use this if the machine comfortably handles ~20GB class models:

- `FAST` -> `ollama:llama3.2:3b`
- `CHAT` -> `ollama:qwen3:30b`
- `CODE` -> `ollama:qwen3-coder:30b`
- `REASON` -> `ollama:qwen3:30b`
- `TOOLS` -> `ollama:qwen3:30b`
- `VISION` -> `ollama:qwen3-vl:30b`
- `ARCHITECT` -> `ollama:mistral-small:24b`
- `OPTIMIZER` -> `ollama:qwen3-coder:30b`

## Telegram + Bitacora Policy

Every model-management action should be observable through both channels.

Mandatory events:

- `model_inventory_verified`
- `model_pull_started`
- `model_pull_completed`
- `model_pull_failed`
- `role_mapping_changed`
- `router_fallback_triggered`
- `role_degraded`
- `role_restored`
- `local_stack_ready`

Expected write path:

1. Write event to Bitacora via `modules.humanoid.ans.evolution_bitacora.append_evolution_log`
2. If severity is `warning` or above, notify Telegram via `modules.humanoid.notify.send_telegram`
3. Mirror summary to the event bus when applicable

## Rollout Order

### Phase 0

- verify local models
- verify Telegram and Bitacora endpoints
- freeze the current baseline

### Phase 1

- download missing models
- validate `ollama list`
- run a smoke prompt against each new model

### Phase 2

- update role mapping via env or config
- keep old models as fallback
- do not remove any existing model yet

### Phase 3

- add model health checks to Bitacora
- add Telegram alerts for missing/failing local models
- validate router fallbacks

### Phase 4

- only after repeated stable runtime, consider pruning weaker local models

## External Terminal Commands

### Check current version

```powershell
ollama --version
ollama list
```

### Upgrade Ollama first if needed

Qwen3-VL requires Ollama `0.12.7+`.

### Conservative set

```powershell
ollama pull qwen3:30b
ollama pull qwen3-coder:30b
ollama pull qwen3-vl:30b
ollama pull mistral-small:24b
```

### Stronger set

```powershell
ollama pull qwen3:30b
ollama pull qwen3-coder:30b
ollama pull qwen3-vl:30b
ollama pull mistral-small:24b
```

## Verification Commands

```powershell
ollama list
ollama run qwen3:30b "ATLAS es mi sistema local en C:\ATLAS_PUSH. No uses referencias externas. Resume en 5 puntos qué roles debería cubrir un stack local de IA para este sistema."
ollama run qwen3-coder:30b "Estoy trabajando en un monorepo local llamado ATLAS en C:\ATLAS_PUSH. Hay componentes como atlas_adapter, nexus, modules, atlas_workspace, memory, scripts y atlas_code_quant. Sin inventar contexto externo, propón un enrutamiento por roles de modelos locales para CHAT, CODE, VISION, REASON, TOOLS y EMBEDDINGS."
ollama run qwen3-vl:30b ".\\screenshot.png" "ATLAS es mi sistema local en C:\ATLAS_PUSH. Analiza esta captura y dime si corresponde a Workspace, Nexus, Robot 3D u otra vista del sistema."
ollama run mistral-small:24b "ATLAS es mi sistema local. Devuélveme solo JSON válido con claves diagnosis, action, confidence, assumptions. Contexto: PUSH=8791, NEXUS=8000, ROBOT=8002, Workspace en /workspace, NEXUS correcto en /nexus."
```

## First Operational Step

```powershell
python scripts/atlas_local_ai_guard.py
python scripts/atlas_local_ai_guard.py --json
python scripts/atlas_local_ai_guard.py --profile conservative
```

## Notes

- Do not remove existing local models until the new stack is verified.
- Keep `atlas_code_quant` changes out of upload/publish flow until the whole integration is finished.
- The correct integration path is additive, observable, and reversible.
