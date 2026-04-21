# Atlas Push · Defectos conocidos

> Registro de defectos detectados durante el Paso 1 de la
> reestructuración. Cada entrada indica si se aborda en este
> refactor o queda fuera de alcance.

## D-001 · Import roto en `modules/atlas_remote_api.py`

- **Archivo**: `modules/atlas_remote_api.py`, línea 7.
- **Defecto**: `from modules.command_router import route_command`.
  La función `route_command` no existe; `command_router` solo
  exporta `handle`. Arrancar `uvicorn modules.atlas_remote_api:app`
  falla en import time.
- **Impacto**: entry-point #4 de la cascada de
  `03_run_atlas_api.ps1` está muerto. Ningún otro archivo vivo
  depende de este módulo.
- **Decisión**: el archivo está fuera de servicio. **No se corrige**
  dentro de esta reestructuración. Será archivado junto con el
  resto de interfaces legadas en una fase posterior, no en los
  pasos A–E. Ratificado en Paso C: el `chore(atlas_push): clean
  atlas_http_api wiring (C)` deliberadamente **no** toca
  `modules/atlas_remote_api.py` (ver `PLAN_STEP_C.md` §4 y §9).
- **Estado**: documentado, sin acción. Se archivará fuera de los
  pasos A–E.

## D-002 · Endpoints duplicados en `atlas_adapter/atlas_http_api.py`

- **Archivo**: `atlas_adapter/atlas_http_api.py`.
- **Defecto**:
  - `@app.get("/modules")` declarado dos veces (líneas ~53 y ~65).
  - `@app.post("/intent")` declarado dos veces (líneas ~85 y ~109).
  - `BaseModel`, `Optional`, `Any`, `time` importados dos veces.
  - La segunda definición de `/intent` es la que queda activa (usa
    `command_router.handle`); la primera es un eco muerto.
- **Impacto**: código confuso, riesgo de que un editor toque la
  declaración muerta. Sin impacto funcional en runtime (FastAPI se
  queda con la última).
- **Decisión**: **se aborda en el Paso C** del plan de refactor, no
  antes. Motivo: queremos tener ya definida la firma futura de
  `IntentRouter` para que la limpieza sea coherente con el destino.
- **Estado**: **cerrado en Paso C** (PR `chore(atlas_push): clean
  atlas_http_api wiring (C)`, commit C1). Las dos definiciones
  `@app.get("/modules")` se consolidan en una sola, la `@app.post("/intent")`
  v1 eco se elimina y los imports duplicados (`BaseModel`,
  `Optional`, `Any`, `time`) quedan en un único bloque. El invariante
  `test_no_duplicate_handlers` en
  `tests/unit/test_http_api_contract.py` blinda la regresión a
  futuro. Ver `PLAN_STEP_C.md` §5 y §6.

## D-003 · Carga del router con ruta Windows absoluta

- **Archivo**: `atlas_adapter/atlas_http_api.py`, líneas 9–18.
- **Defecto**: `ROUTER_PATH = Path(r"C:\ATLAS\modules\command_router.py")`
  + `importlib.util.spec_from_file_location(...)`. La ruta es
  Windows-específica y ya ni siquiera coincide con el layout actual
  (que vive en `C:\ATLAS_PUSH\...`). Más abajo en el mismo archivo
  (línea 102) el router se importa correctamente con
  `from modules.command_router import handle as route_command`.
- **Impacto**: fragilidad de despliegue, imposible de testear fuera
  de Windows, dos formas distintas de resolver el mismo símbolo en
  el mismo archivo.
- **Decisión**: **se aborda en el Paso C** del plan de refactor,
  consolidando todo a `from modules.command_router import handle`.
- **Estado**: **cerrado en Paso C** (PR `chore(atlas_push): clean
  atlas_http_api wiring (C)`, commit C1). Se eliminan
  `import importlib.util`, `from pathlib import Path`, las
  constantes `ATLAS_ROOT`/`ROUTER_PATH` y la función
  `load_handle()`. El router se resuelve con
  `from modules.command_router import handle`, que funciona con
  `uvicorn atlas_adapter.atlas_http_api:app` desde la raíz del
  repo (tal como lo arranca `03_run_atlas_api.ps1`). El invariante
  `test_source_has_no_importlib_or_windows_paths` en
  `tests/unit/test_http_api_contract.py` blinda la regresión a
  futuro. Ver `PLAN_STEP_C.md` §5 y §6.

## D-004 · `core/scheduler.py` es huérfano

- **Archivo**: `core/scheduler.py`.
- **Defecto**: nadie importa `run_scheduler` ni `heartbeat`. Ningún
  script `.ps1` lo arranca. Su único efecto sería escribir
  "ATLAS heartbeat OK" al log cada 10 s si alguien lo lanzara.
- **Impacto**: código muerto no declarado.
- **Decisión**: fuera del perímetro de Atlas Push por decisión del
  owner. **No se toca** en esta reestructuración. Puede archivarse
  junto con el resto de periferia en una fase posterior.
- **Estado**: documentado, sin acción.

## D-005 · `snapshots/` versionado en git

- **Archivo**: directorio `snapshots/` (8 carpetas,
  `20260110_104044_startup` … `20260110_182435_…`).
- **Defecto**: copias completas del código versionadas en git.
  Inflan el repo, confunden a `grep`/`ripgrep`, no son fuentes
  vivas. Generadas por `modules/snapshot_engine.snapshot()`.
- **Impacto**: ruido de repositorio, riesgo bajo de confundir
  snapshot con fuente viva al navegar.
- **Decisión**: **fuera del alcance** de la reestructuración de
  Atlas Push. Se resolverá en un PR de higiene independiente:
  añadir a `.gitignore` y hacer `git rm --cached` del histórico.
- **Estado**: documentado, fuera de alcance.

## D-006 · `.venv/` versionado en git

- **Archivo**: directorio `.venv/`.
- **Defecto**: entorno virtual versionado. Debería estar en
  `.gitignore` y no en el repo.
- **Impacto**: ruido de repositorio, tamaño innecesario.
- **Decisión**: **fuera del alcance** de la reestructuración de
  Atlas Push. Se incluye en el mismo PR de higiene que D-005.
- **Estado**: documentado, fuera de alcance.

## D-007 · Paths hardcodeados a Windows diseminados

- **Archivos**: `atlas.py`, `atlas_runtime.py`, `core/logger.py`,
  `modules/*.py`, `bridge/server.py`,
  `atlas_adapter/atlas_http_api.py`.
- **Defecto**: rutas `C:\ATLAS`, `C:\ATLAS_PUSH`,
  `C:\ATLAS\config\.env` repetidas por todo el código.
- **Impacto**: dificultad de testeo, imposibilidad de ejecutar
  fuera de una máquina Windows específica, duplicación de
  constantes.
- **Decisión**: se aborda **parcialmente** en el Paso E al
  introducir `atlas_push/config/settings.py` como fuente única de
  paths para el nuevo paquete. El código legado se migra
  progresivamente cuando se refactoricen sus archivos.
- **Estado**: documentado, mitigación parcial en Paso E.

## D-008 · Dos routers paralelos (`atlas_runtime.handle` vs `command_router.handle`)

- **Archivos**: `atlas_runtime.py` y `modules/command_router.py`.
- **Defecto**: misma firma `handle(text) -> str`, dos
  implementaciones. `atlas_runtime` es la v1 pobre (solo
  `/status`, `/doctor`, `/modules`). `command_router` es la v2
  completa (añade notas, snapshots, NLP ES).
- **Impacto**: duplicación de contrato, riesgo de divergencia.
- **Decisión**: `atlas_runtime.py` está en la lista de archivables
  aprobada. Se mueve a `legacy/` en el Paso A2.
- **Estado**: documentado, resolución en Paso A2.
