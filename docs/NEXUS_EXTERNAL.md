# NEXUS External Runtime (ATLAS_PUSH)

## Que es
`rest_api.py` es la API real de NEXUS que responde en `http://127.0.0.1:8000`.
En esta instalacion vive fuera de `ATLAS_PUSH`, tipicamente en:

- `C:\ATLAS_NEXUS\atlas_nexus\api\rest_api.py`

## Como se arranca (PowerShell)
Opcion recomendada desde este repo:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\nexus_external_start.ps1
```

Si ya existe `start.ps1` en el runtime externo, el wrapper lo usa.
Si no existe, hace fallback a `python nexus.py --mode api` en el directorio externo.

## Como se verifica

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\nexus_external_status.ps1
curl.exe -i http://127.0.0.1:8000/health
```

Respuesta esperada: `HTTP/1.1 200 OK` con JSON cheap (`ok`, `service`, `ts`, `pid`).

## Instalacion en otra PC
1. Clonar `ATLAS_PUSH`.
2. Definir `NEXUS_ATLAS_PATH` si no usas la ruta por defecto.
3. Ejecutar el wrapper de inicio y luego status.

```powershell
$env:NEXUS_ATLAS_PATH = 'D:\ATLAS_NEXUS\atlas_nexus'
powershell -ExecutionPolicy Bypass -File .\scripts\nexus_external_start.ps1
powershell -ExecutionPolicy Bypass -File .\scripts\nexus_external_status.ps1
```

## Troubleshooting basico
- `404/timeout en /health`: confirmar que `nexus.py --mode api` este levantado en 8000.
- `Ruta no encontrada`: revisar `NEXUS_ATLAS_PATH` y que exista `nexus.py`.
- `Python incorrecto`: probar con `ATLAS_HOOK_PYTHON` o activar `venv` en runtime externo.
