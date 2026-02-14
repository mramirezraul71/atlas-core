# Monitoreo y actualización automática del repositorio

**Tarea asignada:** A partir de ahora esta tarea es responsabilidad del sistema de monitoreo del repo. El script y la configuración siguientes son los encargados de mantener el repositorio monitoreado y actualizado (por ciclos o tras arreglos).

## Componentes

| Componente | Descripción |
|------------|-------------|
| **config/repo_monitor.yaml** | Configuración: filtros Git, ciclo (intervalo por defecto 10 min), after-fix, bitácora, acciones ante errores. |
| **scripts/repo_monitor.py** | Lógica: fetch, status, pull, commit+push con filtros, reintentos; **todas las tareas se envían a la Bitácora ANS**. |
| **scripts/run_repo_monitor.ps1** | Wrapper PowerShell: invoca el script con filtros, comandos y manejo de códigos de salida. |
| **logs/repo_monitor.log** | Log rotativo (tamaño y copias definidos en config). |
| **Bitácora ANS** | En el dashboard (Bitácora ANS): entradas con problema "Monitor repo" y detalle del mensaje (ciclo, after-fix, errores). |

## Uso

### Por ciclos (monitoreo periódico)

- **Un ciclo (manual o programado):**  
  `scripts\run_repo_monitor.ps1 -Cycle`  
  Equivalente: `python scripts/repo_monitor.py --cycle`

- Hace: `git fetch`, comprueba estado (branch, head, remote), aplica filtros a `git status` y, si está configurado, puede hacer `pull` (o solo reportar "update available").

- **Ejecución automática por ATLAS:**  
  Al arrancar el servidor ATLAS (con `SCHED_ENABLED=true`), se registra un job del scheduler llamado `repo_monitor_cycle` que ejecuta `scripts/repo_monitor.py --cycle` cada `cycle.interval_seconds` segundos (por defecto 600). No hace falta Programador de tareas de Windows: el ciclo corre en segundo plano. Ver `modules/humanoid/scheduler/repo_monitor_jobs.py` y `runner.py` (kind `repo_monitor_cycle`).

- **Programar ciclos (Windows, sin ATLAS):**  
  Si no usas el servidor ATLAS, Programador de tareas: ejecutar `powershell.exe -File "C:\ATLAS_PUSH\scripts\run_repo_monitor.ps1" -Cycle` con la periodicidad deseada.

### Tras arreglos (commit + push)

- **Commit y push de cambios filtrados:**  
  `scripts\run_repo_monitor.ps1 -AfterFix`  
  Con mensaje:  
  `scripts\run_repo_monitor.ps1 -AfterFix -Message "fix: descripción"`

- Solo se añaden y commitean archivos que **no** estén en `git.exclude_paths`. Si se define `include_paths_for_commit`, solo se consideran rutas que coincidan con esos patrones.

### Solo estado

- **Consultar estado sin modificar nada:**  
  `scripts\run_repo_monitor.ps1 -StatusOnly`

## Filtros Git

- **exclude_paths:** Rutas que nunca se incluyen en status filtrado ni en commit automático (además de lo que ya ignora `.gitignore`). Se pueden usar patrones con `*` (ej. `snapshots/ans/*.md`).
- **include_paths_for_commit:** En modo after-fix, si no está vacío, solo se commitean cambios en rutas que coincidan con estos patrones.

Los filtros se aplican en `repo_monitor.py` sobre la salida de `git status --short`.

## Acciones ante errores

Configuradas en `config/repo_monitor.yaml` bajo `on_error`:

| Error | Opciones `then` | Comportamiento |
|-------|------------------|----------------|
| **fetch_fail** | `log_and_continue` | Tras N reintentos, solo registra y sigue. |
| | `reset_to_remote` | Tras reintentos, ejecuta `git fetch` y `git reset --hard remote/branch` (usar con cuidado). |
| **pull_fail** | `log_and_abort` | Tras reintentos, registra y termina con código de error (no se hace reset para no perder trabajo local). |
| **push_fail** | `log_and_abort` | Tras reintentos, registra y termina con código de error. |

Reintentos y retardos se configuran con `retries` y `delay_seconds` en cada bloque.

## Comandos Git utilizados

- Ciclo: `git fetch <remote>`, `git rev-parse HEAD`, `git rev-parse <remote>/<branch>`, `git status --short`, opcionalmente `git pull --no-edit <remote> <branch>`.
- After-fix: `git add <path>`, `git commit -m <msg>`, `git push <remote> <branch>`.
- Recuperación (si `fetch_fail.then == reset_to_remote`): `git reset --hard <remote>/<branch>`.

Todo se ejecuta desde la raíz del repo (`ATLAS_REPO_PATH` o raíz detectada por el script).

## Bitácora ANS

Todas las tareas del monitor se reflejan en la **Bitácora ANS** del dashboard (si PUSH está en marcha y `bitacora.enabled` es true en config):

- **Ciclo iniciado / finalizado** (branch, head, remote, has_update, changed).
- **Fetch o pull fallido** (con detalle del error).
- **After-fix iniciado / push OK / push o commit fallido** (rama, resultado).
- **Status** (solo si se ejecuta `--status-only`).

En la Bitácora aparecen como problema **"Monitor repo"** y el detalle es el mensaje. La URL del dashboard se configura en `config/repo_monitor.yaml` → `bitacora.dashboard_url` (por defecto `http://127.0.0.1:8791`). Si el dashboard no está disponible, el script sigue funcionando y solo registra en `logs/repo_monitor.log`.

## Configuración de ejemplo

- **Solo monitoreo (report_only):** `cycle.report_only: true`, `cycle.pull_on_cycle: false` → solo fetch y log de "update available".
- **Pull automático en ciclo:** `cycle.pull_on_cycle: true`, `cycle.report_only: false`.
- **Rama de push distinta:** `repo.push_branch: "intent-input-rename"` para hacer push a esa rama en after-fix.

## Códigos de salida

- **0** — OK.
- **1** — Error en la operación (fetch, pull, commit o push tras reintentos).
- **2** — No es un repositorio Git o error de configuración.

El wrapper PowerShell propaga el código de salida y escribe un mensaje en amarillo si el código no es 0.

## Subir repo de otras apps (Cursor y chat)

Cuando se solicita por **Cursor** (objetivo o paso del plan) o por **chat** (comando o lenguaje natural), el sistema puede subir el repo de **esta** app o de **otras** definidas en `known_apps`.

### Config: known_apps

En `config/repo_monitor.yaml`:

```yaml
known_apps:
  atlas_push: null          # null = raíz actual
  atlas_nexus: "nexus/atlas_nexus"
  robot: "nexus/atlas_nexus_robot"
  # más: "mi_app": "C:/ruta/al/repo"
```

### Desde Cursor

- Objetivo tipo: *"Sube el repo de atlas_nexus"* o *"Haz push del repositorio robot"*.
- El plan puede generar un paso como *"Subir repo de atlas_nexus"*; en modo auto se ejecuta el push de ese repo.

### Desde el chat (intent/command_router)

- **Comando:** `/repo-push` (repo actual) o `/repo-push atlas_nexus`.
- **Lenguaje natural:** *"Sube el repo"*, *"Sube el repo de atlas_nexus"*, *"Haz push del repo de robot"*.
- **Listar apps:** *"Qué repos puedo subir"*, *"Lista de repos"* → devuelve known_apps.

### API

- **POST /api/repo/push** — Body: `{ "app_id": "atlas_nexus", "message": "opcional" }` o `{ "repo_path": "C:\\ruta" }`.
- **GET /api/repo/apps** — Lista de known_apps (app_id → ruta).

Lógica compartida: `modules/repo_push.py` (push_repo, resolve_path, list_known_apps).
