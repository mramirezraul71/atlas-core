# Service Venvs (Push / Nexus / Robot)

## Objetivo
Separar dependencias por servicio para reducir conflictos de paquetes en Windows.

## Estructura
- `.venv_push` -> `requirements.txt` + `constraints/push-base.txt`
- `.venv_nexus` -> `nexus/atlas_nexus/requirements.txt` + `constraints/nexus-base.txt`
- `.venv_robot` -> `nexus/atlas_nexus_robot/backend/requirements.txt` + `constraints/robot-base.txt`

## Bootstrap
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\windows\setup_service_venvs.ps1
```

## Notas
- No reemplaza el flujo actual; agrega una ruta estable para entornos aislados.
- Si un servicio necesita ajustes, actualizar su archivo `constraints/*-base.txt`.
