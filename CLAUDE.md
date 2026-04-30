# ATLAS_PUSH — Claude Code Context

## Project Overview
**ATLAS Core** v0.6.0 — Multi-agent AI system ("Hands of the RAULI Robot").
Windows-native runtime. Manages tools, runners, memory, audit, and cognitive architecture for the RAULI robot.

## Architecture
- **NEXUS** (`nexus/`): Body/hardware interface, HTTP API on port 8000. Must be running before `main.py`.
- **Orchestrator** (`main.py`, `orchestrator.py`): Multi-agent entrypoint. Waits for NEXUS health before starting.
- **Agents** (`agents/`): `coder_agent.py`, `inspector_agent.py`, `navigator_agent.py`, `orchestrator.py`
- **Brain** (`brain/`): Cognitive architecture (memory, NLU, vision, audio)
- **Core** (`core/`): Logger, scheduler, shared utilities
- **Atlas Adapter** (`atlas_adapter/`): Bridge layer
- **Memory Engine** (`memory_engine/`): ChromaDB + FAISS + Mem0 + LangChain
- **Evolution Daemon** (`evolution_daemon.py`): Background learning/autonomy loop

## Critical Ports
- `8000` — NEXUS health/API (required before startup)
- `8791` — ATLAS API
- `8002` — Secondary service

## Key Files
- `main.py` — Entry point, waits for NEXUS, boots orchestrator
- `atlas.py` — Core ATLAS logic
- `config/atlas.env` — Environment config (loaded at startup)
- `requirements.txt` — Python dependencies
- `ATLAS_SUPERVISOR.md` — Canonical operational rules

## Stack
- **Runtime**: Python 3.x, FastAPI, Uvicorn, WebSockets
- **AI/ML**: PyTorch, YOLOv8 (ultralytics), OpenAI Whisper, Ollama, OpenAI API
- **Memory**: ChromaDB, FAISS, sentence-transformers, Mem0, LangChain, LangFlow
- **Vision**: OpenCV, timm, torchvision
- **Audio**: sounddevice, webrtcvad
- **Messaging**: pyzmq
- **Deployment**: Docker, docker-compose, Blue-Green + Canary (see `docs/DEPLOY.md`)

## Setup & Run
```powershell
# Full setup
.\INICIAR.ps1

# Step by step
.\00_prereqs.ps1
.\01_setup_venv.ps1
.\02_install_deps.ps1
.\03_run_atlas_api.ps1

# Smoke tests
.\04_smoke_tests.ps1

# Kill port if stuck
.\05_kill_port.ps1
```

## Operational Rules (from ATLAS_SUPERVISOR.md)
1. No mass file deletion.
2. No destructive commands (`rm -rf`, `del /s`, `format`, etc.).
3. Stay within `C:\ATLAS_PUSH` workspace.
4. Maintain existing APIs and contracts.
5. Prefer small, auditable changes with backups.
6. Write reports to `/reports`, logs to `/logs`.

## Development Notes
- Environment credentials expected at `C:\dev\credenciales.txt` (for Evolution daemon).
- Use `venv/` for Python environment.
- Test files: `conftest.py`, `pytest.ini`, `tests/`, `test_*.py`.
- Logs: `debug.log`, `logs/`.
- Cycle: **Diagnose → Short Plan → Safe Execution → Final Report**.
- If risk is high, request explicit confirmation before proceeding.
