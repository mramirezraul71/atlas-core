# ATLAS Quality System (Sistema de Calidad)

## Descripción General

El **Sistema de Calidad de ATLAS** es el módulo que garantiza que todas las operaciones internas del robot se ejecuten siguiendo **Procedimientos Operacionales de Trabajo (POTs)** estandarizados.

Cada POT actúa como una **guía tutorial** que ATLAS sigue para ejecutar tareas específicas, desde commits de código hasta reparaciones de hardware.

**CONFIABILIDAD: 100%** - Sistema completamente autónomo.

## Arquitectura Completa de Autonomía

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        ATLAS QUALITY SYSTEM v2.0                            │
│                     (SISTEMA DE AUTONOMIA COMPLETA)                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                  │
│  │   TRIGGERS   │───▶│  DISPATCHER  │───▶│   EXECUTOR   │                  │
│  │  (triggers)  │    │ (dispatcher) │    │  (executor)  │                  │
│  │              │    │              │    │              │                  │
│  │ - Git Changes│    │ - Queue      │    │ - Run Steps  │                  │
│  │ - Git Behind │    │ - Priority   │    │ - Rollback   │                  │
│  │ - Svc Down   │    │ - Auto-select│    │ - Reports    │                  │
│  │ - Disk Full  │    │ - Approval   │    │ - Snapshots  │                  │
│  │ - Incidents  │    │              │    │              │                  │
│  └──────────────┘    └──────────────┘    └──────────────┘                  │
│         ▲                   │                   │                           │
│         │                   ▼                   ▼                           │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                  │
│  │     ANS      │    │   REGISTRY   │    │    POTs      │                  │
│  │ (incidents)  │    │  (registry)  │    │   (18 proc)  │                  │
│  │              │    │              │    │              │                  │
│  │ - Create     │    │ - Search     │    │ - Git        │                  │
│  │ - Resolve    │    │ - Match      │    │ - Repair     │                  │
│  │ - Auto-heal  │    │ - Keywords   │    │ - Maintain   │                  │
│  └──────────────┘    └──────────────┘    │ - Autonomy   │                  │
│         │                                 │ - Update     │                  │
│         ▼                                 └──────────────┘                  │
│  ┌──────────────┐                               │                           │
│  │  SCHEDULER   │                               ▼                           │
│  │   (jobs)     │                        ┌──────────────┐                  │
│  │              │                        │   REPORTS    │                  │
│  │ - pot_execute│                        │  (reports/)  │                  │
│  │ - autonomy   │                        │              │                  │
│  │ - daily_maint│                        │ - JSON logs  │                  │
│  │ - git_sync   │                        │ - Metrics    │                  │
│  └──────────────┘                        └──────────────┘                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Flujo de Autonomía

```
╔═══════════════════════════════════════════════════════════════════════════╗
║                         FLUJO DE EJECUCIÓN AUTÓNOMA                        ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║  1. DETECCIÓN                                                             ║
║     ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐                ║
║     │ Trigger │   │   ANS   │   │Scheduler│   │   API   │                ║
║     │ Engine  │   │Incident │   │  Job    │   │ Request │                ║
║     └────┬────┘   └────┬────┘   └────┬────┘   └────┬────┘                ║
║          │             │             │             │                      ║
║          └─────────────┴──────┬──────┴─────────────┘                      ║
║                               ▼                                           ║
║  2. DESPACHO                                                              ║
║     ┌───────────────────────────────────────────┐                        ║
║     │            POT DISPATCHER                  │                        ║
║     │                                           │                        ║
║     │  • Encolar request                        │                        ║
║     │  • Seleccionar POT (auto o específico)    │                        ║
║     │  • Verificar aprobación si es crítico     │                        ║
║     │  • Pasar a executor                       │                        ║
║     └───────────────────────────────────────────┘                        ║
║                               │                                           ║
║                               ▼                                           ║
║  3. EJECUCIÓN                                                             ║
║     ┌───────────────────────────────────────────┐                        ║
║     │            POT EXECUTOR                    │                        ║
║     │                                           │                        ║
║     │  • Ejecutar steps secuencialmente         │                        ║
║     │  • Manejar retries y timeouts             │                        ║
║     │  • Ejecutar rollback si falla             │                        ║
║     │  • Guardar reporte JSON                   │                        ║
║     └───────────────────────────────────────────┘                        ║
║                               │                                           ║
║                               ▼                                           ║
║  4. NOTIFICACIÓN                                                          ║
║     ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐                ║
║     │Telegram │   │Dashboard│   │   ANS   │   │ OPS Bus │                ║
║     │  Notify │   │ Refresh │   │ Resolve │   │  Event  │                ║
║     └─────────┘   └─────────┘   └─────────┘   └─────────┘                ║
║                                                                           ║
╚═══════════════════════════════════════════════════════════════════════════╝
```

## Componentes

### 1. POTs (Procedimientos Operacionales de Trabajo)

Los POTs son scripts declarativos que definen cómo ejecutar una tarea:

| Categoría | POTs | Descripción |
|-----------|------|-------------|
| **Deployment** | `git_commit`, `git_push`, `git_pull`, `deployment_full` | Operaciones Git y despliegue |
| **Upgrade** | `repo_update` | Actualización de repositorio y dependencias |
| **Repair** | `camera_repair`, `services_repair`, `api_repair` | Reparación de componentes |
| **Maintenance** | `maintenance_daily`, `maintenance_weekly` | Mantenimiento preventivo |
| **Incident** | `incident_triage`, `incident_response` | Gestión de incidentes |
| **Diagnostic** | `diagnostic_full` | Diagnóstico completo del sistema |
| **Session** | `session_startup`, `session_shutdown` | Inicio y cierre de sesión |
| **Communication** | `notification_broadcast` | Difusión a todos los canales |
| **Autonomy** | `autonomy_full_cycle` | Ciclo completo de autonomía (health, healing, learning) |
| **Auto-Update** | `auto_update_full` | Actualización automática (Tríada PyPI/GitHub/HF, cuarentena) |

### 2. Executor Engine

Motor que ejecuta cada paso del POT:

- **Tipos de pasos soportados:**
  - `COMMAND`: Comando de shell
  - `SCRIPT`: Script Python
  - `HTTP`: Llamada HTTP/API
  - `CHECK`: Verificación de condición
  - `WAIT`: Espera de tiempo
  - `LOG`: Registro en log
  - `NOTIFY`: Notificación a canales
  - `SNAPSHOT`: Captura de estado
  - `CONFIRM`: Punto de confirmación
  - `ROLLBACK`: Paso de reversión

### 3. Sync Engine

Motor de sincronización automática:

```python
from modules.humanoid.quality import sync_operation

# Ejecutar operación con su POT asociado
result = sync_operation("commit")  # -> Ejecuta git_commit POT

# Operaciones mapeadas (43+):
# commit, push, pull, sync, update, deploy, startup, shutdown, etc.
```

### 4. Cerebro Connector

Integración con todos los sistemas ATLAS:

```python
from modules.humanoid.quality import get_bridge

bridge = get_bridge()

# Se conecta automáticamente con:
# - ANS (Sistema Nervioso Autónomo) - Bitácora e incidentes
# - Dashboard - Actualización visual
# - Canales - Telegram, OPS Bus
```

## Flujo de Ejecución

```
┌─────────────────────────────────────────────────────────────────────┐
│                         FLUJO DE EJECUCIÓN POT                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  TRIGGER (Incidente/Comando/Scheduler)                              │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────┐                                                    │
│  │  Registry   │  Busca POT por trigger_check_ids o keywords        │
│  │   Match     │                                                     │
│  └──────┬──────┘                                                    │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────┐                                                    │
│  │   Cerebro   │  Registra inicio: bridge.on_pot_start()            │
│  │   Notify    │  → Log en ANS                                       │
│  └──────┬──────┘  → Notifica a OPS Bus                              │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────────────────────────────────────┐                    │
│  │              EXECUTOR ENGINE                 │                    │
│  │                                              │                    │
│  │  Para cada STEP:                            │                    │
│  │    1. Verificar condiciones (skip_if, if)   │                    │
│  │    2. Ejecutar (cmd/script/http/check)      │                    │
│  │    3. Capturar output → contexto            │                    │
│  │    4. Evaluar check_expression              │                    │
│  │    5. Retry si falló (hasta N intentos)     │                    │
│  │    6. Continue o Stop según config          │                    │
│  │                                              │                    │
│  │  Si FALLO y has_rollback:                   │                    │
│  │    → Ejecutar rollback_steps                │                    │
│  └──────────────────────────────────────────────┘                    │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────┐                                                    │
│  │   Report    │  Genera JSON con resultado completo                │
│  │   Generate  │  → modules/humanoid/quality/reports/               │
│  └──────┬──────┘                                                    │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────┐                                                    │
│  │   Cerebro   │  Registra finalización: bridge.on_pot_complete()   │
│  │   Complete  │  → Log en ANS                                       │
│  └──────┬──────┘  → Si fallo → Crear incidente                      │
│         │         → Notifica Telegram si severity alta              │
│         │                                                            │
│         ▼                                                            │
│  ┌─────────────┐                                                    │
│  │  Auto-Sync  │  Si operación en AUTO_SYNC_OPERATIONS:             │
│  │  (optional) │    → Ejecutar git commit + push automático         │
│  └─────────────┘                                                    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Estados de Reporte

Cada ejecución de POT genera un reporte con estos estados:

| Estado | Emoji | Descripción |
|--------|-------|-------------|
| `success` | ✅ | Todos los pasos completados exitosamente |
| `partial` | ⚠️ | Algunos pasos fallaron pero el POT continuó |
| `failed` | ❌ | El POT falló y se detuvo |
| `rollback_ok` | 🔄 | El POT falló pero el rollback fue exitoso |
| `rollback_fail` | 💥 | El POT y el rollback fallaron |

## Operaciones Mapeadas

El Sync Engine mapea **43+ operaciones** a sus POTs:

```python
OPERATION_POT_MAP = {
    # Git
    "commit": "git_commit",
    "push": "git_push",
    "pull": "git_pull",
    "sync": "git_pull",

    # Repo
    "update": "repo_update",
    "upgrade": "repo_update",

    # Deploy
    "deploy": "deployment_full",
    "release": "deployment_full",

    # Session
    "startup": "session_startup",
    "buenos_dias": "session_startup",
    "shutdown": "session_shutdown",
    "buenas_noches": "session_shutdown",

    # Repairs
    "repair_camera": "camera_repair",
    "repair_services": "services_repair",
    "repair_api": "api_repair",

    # Maintenance
    "maintenance": "maintenance_daily",
    "weekly_maintenance": "maintenance_weekly",

    # Incidents
    "triage": "incident_triage",
    "respond": "incident_response",

    # Diagnostics
    "diagnostic": "diagnostic_full",
    "health_check": "diagnostic_full",
}
```

## API Endpoints

El módulo expone endpoints REST:

| Endpoint | Método | Descripción |
|----------|--------|-------------|
| `/quality/pots` | GET | Listar todos los POTs |
| `/quality/pots/{id}` | GET | Obtener POT específico |
| `/quality/pots/execute` | POST | Ejecutar un POT |
| `/quality/pots/match` | POST | Buscar POT para un incidente |
| `/quality/reports` | GET | Listar reportes de ejecución |
| `/quality/categories` | GET | Listar categorías de POT |
| `/quality/tutorial/{id}` | GET | Obtener tutorial de un POT |

## Uso Programático

### Ejecutar un POT

```python
from modules.humanoid.quality import get_pot, execute_pot

# Obtener POT
pot = get_pot("git_commit")

# Ejecutar con contexto
result = execute_pot(
    pot,
    context={"commit_message": "feat: nueva funcionalidad"},
    dry_run=False,
    sync_to_cerebro=True,  # Registrar en ANS
    notify_on_complete=True,  # Notificar a Telegram
)

print(f"OK: {result.ok}, Steps: {result.steps_ok}/{result.steps_total}")
```

### Usar Sync Operation

```python
from modules.humanoid.quality import sync_operation

# Auto-detecta el POT correcto y lo ejecuta
result = sync_operation("commit")
```

### Obtener POT para Incidente

```python
from modules.humanoid.quality import get_pot_by_incident

# Busca el mejor POT según el check_id y mensaje del incidente
pot = get_pot_by_incident(
    check_id="camera_stream",
    message="Camera index 0 not responding"
)
# Retorna: camera_repair POT
```

## Estructura de Archivos

```
modules/humanoid/quality/
├── __init__.py           # Exports principales
├── models.py             # Modelos (POT, POTStep, POTResult)
├── registry.py           # Búsqueda y listado de POTs
├── executor.py           # Motor de ejecución
├── sync_engine.py        # Sincronización automática
├── cerebro_connector.py  # Conexión con ANS/Dashboard
├── api.py                # Endpoints FastAPI
├── pots/                 # Definiciones de POTs
│   ├── __init__.py       # Registry de POTs
│   ├── git_commit.py
│   ├── git_push.py
│   ├── git_pull.py
│   ├── repo_update.py
│   ├── deployment_full.py
│   ├── camera_repair.py
│   ├── services_repair.py
│   ├── api_repair.py
│   ├── maintenance_daily.py
│   ├── maintenance_weekly.py
│   ├── incident_triage.py
│   ├── incident_response.py
│   ├── diagnostic_full.py
│   ├── notification_broadcast.py
│   ├── session_startup.py
│   └── session_shutdown.py
├── reports/              # Reportes de ejecución (JSON)
└── snapshots/            # Snapshots de estado
```

## Integración con Workshop Central

El **Workshop Central** (`scripts/atlas_central_workshop.py`) usa el Sistema de Calidad para seleccionar y ejecutar POTs automáticamente:

```python
# En Workshop Central
from modules.humanoid.quality import get_pot_by_incident, execute_pot

# Para cada incidente
pot = get_pot_by_incident(incident["check_id"], incident["message"])
if pot:
    result = execute_pot(pot, context={"incident_id": incident["id"]})
```

## Variables de Entorno

| Variable | Default | Descripción |
|----------|---------|-------------|
| `QUALITY_POT_TIMEOUT_DEFAULT` | 300 | Timeout por defecto de POTs (seg) |
| `QUALITY_POT_DRY_RUN` | false | Ejecutar en modo simulación |
| `QUALITY_NOTIFY_ENABLED` | true | Habilitar notificaciones |
| `QUALITY_DEBUG` | false | Log detallado por paso |

## Mejores Prácticas

1. **POTs Atómicos**: Cada POT debe hacer UNA cosa bien definida
2. **Rollback Siempre**: POTs críticos deben tener `rollback_steps`
3. **Timeouts Realistas**: Configurar `timeout_seconds` apropiados
4. **Continue on Failure**: Usar para pasos opcionales
5. **Capture Output**: Habilitar para debugging y auditoría
6. **Notificaciones**: Solo para eventos importantes (evitar spam)

## Creación de Nuevos POTs

Para crear un nuevo POT:

1. Crear archivo en `modules/humanoid/quality/pots/mi_pot.py`
2. Definir función `get_pot() -> POT`
3. Registrar en `pots/__init__.py`
4. (Opcional) Agregar operaciones al `OPERATION_POT_MAP` en `sync_engine.py`

Ejemplo mínimo:

```python
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType

def get_pot() -> POT:
    return POT(
        id="mi_pot",
        name="Mi Procedimiento",
        description="Descripción del procedimiento",
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS",

        steps=[
            POTStep(
                id="paso_1",
                name="Primer paso",
                description="Descripción del paso",
                step_type=StepType.COMMAND,
                command="echo 'Hola ATLAS'",
            ),
        ],
    )
```

---

*Sistema de Calidad ATLAS - Versión 2.0*
