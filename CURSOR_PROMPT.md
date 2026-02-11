# Cursor Prompt — ATLAS Instalación + Checks (Windows)

Eres Cursor actuando como asistente técnico en una PC con Windows. Debes ejecutar comandos PowerShell y reportar resultados.

## Objetivo
- Preparar entorno en `C:\ATLAS_PUSH`
- Crear venv `.venv`
- Instalar dependencias mínimas
- Levantar ATLAS API en `127.0.0.1:8791`
- Verificar `GET /status` y `GET /modules` devuelven 200 OK
- Si el puerto 8791 está ocupado, matar el PID dueño del puerto y reintentar.

## Pasos
1) Ejecuta prereqs:
```powershell
Set-ExecutionPolicy -Scope Process Bypass -Force
cd C:\ATLAS_TOOLS
.\00_prereqs.ps1 -RepoPath C:\ATLAS_PUSH
```

2) Setup venv:
```powershell
.\01_setup_venv.ps1 -RepoPath C:\ATLAS_PUSH
```

3) Install deps:
```powershell
.\02_install_deps.ps1 -RepoPath C:\ATLAS_PUSH
```

4) Levantar ATLAS:
- Preferir: `atlas_adapter.atlas_http_api:app`
```powershell
.\03_run_atlas_api.ps1 -RepoPath C:\ATLAS_PUSH -AtlasPort 8791
```

5) Smoke tests:
```powershell
.\04_smoke_tests.ps1 -RepoPath C:\ATLAS_PUSH -AtlasPort 8791
```

## Reporte esperado
- Mostrar outputs relevantes (Python version, pip, uvicorn start lines)
- Confirmar JSON de /status y /modules
- Si falla, pegar el error EXACTO y el comando que lo provocó.
