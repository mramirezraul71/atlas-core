# Stack PUSH + Quant + Radar

Arranque controlado del motor **Atlas Code Quant** (`:8792`) y del adaptador **PUSH** (`:8791`) para que el **Radar institucional** (`/radar/dashboard`) use datos vivos (SSE Bloque 4, summary, búsqueda de símbolos).

## Requisitos

- Windows, PowerShell 5.1+.
- Repo en `C:\ATLAS_PUSH` (o ajusta `-RepoRoot` si añades el parámetro al script).
- Python del repo: preferible `venv\Scripts\python.exe` (el script usa `scripts\atlas_runtime.ps1` para resolver intérprete).
- Variable de sesión **`ATLAS_QUANT_API_KEY`**: misma clave en Quant y en PUSH (Quant valida `X-Api-Key`; PUSH la reenvía a Quant).

## Script de arranque

```powershell
cd C:\ATLAS_PUSH
$env:ATLAS_QUANT_API_KEY = "tu-clave-compartida"
.\scripts\start_atlas_radar_stack.ps1
```

Sin cámara (solo mercado / sin hardware de visión):

```powershell
.\scripts\start_atlas_radar_stack.ps1 -DisableCamera
```

Si ya liberaste puertos a mano:

```powershell
.\scripts\start_atlas_radar_stack.ps1 -SkipKill
```

### Qué hace el script

1. Comprueba `atlas_code_quant`, `atlas_adapter` y `ATLAS_QUANT_API_KEY`.
2. Libera `:8792` y `:8791` (`free_port.ps1 -Kill`) y detiene procesos Python típicos (PUSH `atlas_http_api`, Quant `main.py`).
3. Arranca **Quant** desde `atlas_code_quant` con `PYTHONPATH=C:\ATLAS_PUSH`, `ENABLE_CAMERA`, logs en `logs/quant.log` y `logs/quant.stderr.log`.
4. Arranca **PUSH** con `python -m atlas_adapter.atlas_http_api`, `QUANT_BASE_URL=http://127.0.0.1:8792`, `ATLAS_RADAR_QUANT_HTTP=1`, logs en `logs/push.log` y `logs/push.stderr.log`.
5. Smoke: `/camera/health` en Quant, `/api/radar/symbols/search`, `/api/radar/dashboard/summary` (sin `demonstration_without_engine`), primeros bytes de SSE con `"sequence"`, y `/api/radar/camera/status` en PUSH si existe.

Códigos de salida distintos de `0` indican fallo (ver mensaje y logs).

## Variables de entorno (referencia)

| Variable | Dónde | Uso |
|----------|--------|-----|
| `ATLAS_QUANT_API_KEY` | Sesión / proceso | Autenticación hacia Quant y radar→Quant |
| `PYTHONPATH` | `C:\ATLAS_PUSH` | Imports `atlas_adapter` y paquetes del repo |
| `ENABLE_CAMERA` | Quant | `true`/`false` según hardware |
| `QUANT_BASE_URL` | PUSH | Base HTTP del motor (p. ej. `http://127.0.0.1:8792`) |
| `ATLAS_RADAR_QUANT_HTTP` | PUSH | `1` para forzar cliente HTTP radar→Quant |
| `ATLAS_PUSH_HOST` / `ATLAS_PUSH_PORT` | Opcional | Host/puerto del `uvicorn` embebido en `python -m atlas_adapter.atlas_http_api` (por defecto `127.0.0.1:8791`) |

## URLs

- **V4 (landing):** http://127.0.0.1:8791/ui  
- **Radar:** http://127.0.0.1:8791/radar/dashboard  

Desde V4, tarjeta y chip **Radar Institucional** abren el radar en la **misma pestaña** (`/radar/dashboard`).

## Troubleshooting

| Síntoma | Acción |
|---------|--------|
| `demonstration_without_engine` en summary | Quant no accesible desde PUSH: comprobar `QUANT_BASE_URL`, firewall, y que Quant arrancó sin error en `logs/quant.stderr.log`. |
| `symbols/search` → 404 | PUSH no es el código actual: reiniciar con el script; comprobar que solo un proceso escucha `:8791`. |
| SSE sin `"sequence"` | Instancia PUSH antigua; reiniciar tras `git pull`. |
| `ATLAS_QUANT_API_KEY` ausente | Definir en la misma sesión de PowerShell **antes** de ejecutar el script. |
| Puerto ocupado tras fallos | `.\scripts\free_port.ps1 -Port 8791 -Kill` y `8792`. |

## Arranque manual (sin script)

**Terminal 1 — Quant**

```powershell
cd C:\ATLAS_PUSH\atlas_code_quant
$env:PYTHONPATH = "C:\ATLAS_PUSH"
$env:ATLAS_QUANT_API_KEY = "..."
$env:ENABLE_CAMERA = "true"
..\venv\Scripts\python.exe main.py
```

**Terminal 2 — PUSH**

```powershell
cd C:\ATLAS_PUSH
$env:PYTHONPATH = "C:\ATLAS_PUSH"
$env:QUANT_BASE_URL = "http://127.0.0.1:8792"
$env:ATLAS_QUANT_API_KEY = "..."
$env:ATLAS_RADAR_QUANT_HTTP = "1"
.\venv\Scripts\python.exe -m atlas_adapter.atlas_http_api
```

Alternativa equivalente: `python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791`.
