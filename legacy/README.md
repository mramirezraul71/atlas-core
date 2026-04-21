# legacy/

Esta carpeta contiene archivos **archivados** que se mantienen en el repositorio
únicamente como referencia histórica. **No forman parte del perímetro vivo de
Atlas Push** y no deben ser importados ni ejecutados por código nuevo.

## Contexto

En la reestructuración de Atlas Push (ver `docs/atlas_push/ARCHITECTURE.md`) se
identificaron varios módulos en la raíz del repositorio que constituían puntos
de entrada paralelos o precursores al perímetro vivo actual. Para reducir la
superficie activa del proyecto sin perder trazabilidad, esos ficheros se
desplazaron aquí en el PR A2 (`chore(atlas_push): archive 4 legacy files and
simplify start script (A2)`).

El punto de entrada HTTP vivo es, en exclusiva:

- `atlas_adapter.atlas_http_api:app` (uvicorn, `127.0.0.1:8791`)

Lanzado por `03_run_atlas_api.ps1` (cascada colapsada en A2).

## Contenido archivado

| Archivo | Origen previo | Motivo de archivado |
|---|---|---|
| `atlas.py` | raíz del repo | Precursor minimalista del runtime. Sustituido por `atlas_runtime.py` y posteriormente por `atlas_adapter.atlas_http_api`. Sin consumidores vivos en el perímetro. |
| `atlas_runtime.py` | raíz del repo | Router v1 (`route_command`) con intents hardcoded. Sustituido por `modules/command_router.py` (v2). Sin consumidores en el perímetro HTTP actual. |
| `bridge/atlas_api_min.py` | `bridge/` | API HTTP mínima paralela (FastAPI) que duplicaba superficie con `atlas_adapter.atlas_http_api`. No referenciada por `03_run_atlas_api.ps1` ni por el adaptador vivo. |
| `atlas_adapter/run_atlas_api.cmd` | `atlas_adapter/` | Lanzador `.cmd` alternativo. Sustituido por `03_run_atlas_api.ps1` como único script de arranque oficial. |

## Reglas

1. **No importar** desde `legacy/` en código nuevo o en el perímetro vivo
   (`atlas_adapter/`, `modules/`, `core/`, `runtime/`, `bridge/` —lo que queda
   de `bridge/`— y el futuro `atlas_push/`).
2. **No añadir** archivos nuevos aquí salvo como parte de un archivado
   explícitamente aprobado siguiendo el mismo patrón (PR dedicado,
   entrada documentada en esta tabla).
3. **No ejecutar** estos ficheros como procesos vivos. Si se necesita
   consultar comportamiento histórico, hacerlo en modo lectura.
4. **Eliminación futura**: la carpeta podrá eliminarse en un PR posterior
   una vez que la migración a `atlas_push/` (pasos B–E del plan) esté
   completa y validada. Hasta entonces, se conserva por seguridad.

## Referencias

- `docs/atlas_push/ARCHITECTURE.md` — diseño objetivo.
- `docs/atlas_push/CURRENT_STATE.md` — descripción del estado anterior a A2.
- `docs/atlas_push/DEFECTS.md` — defectos documentados (no corregidos en A2).
