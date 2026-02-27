# ATLAS Quality Module - POTs (Procedimientos Operacionales de Trabajo)

## Descripción

El módulo de **Calidad (QA/QC)** proporciona un sistema estandarizado de **POTs (Procedimientos Operacionales de Trabajo)** que actúan como tutoriales ejecutables para que ATLAS realice cada operación de forma consistente y documentada.

## Arquitectura

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ATLAS Quality Module                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │    Registry     │    │    Executor     │    │      API        │         │
│  │                 │    │                 │    │                 │         │
│  │ • list_pots()   │───▶│ • execute_pot() │◀───│ • /quality/*    │         │
│  │ • get_pot()     │    │ • execute_step()│    │ • /pots/*       │         │
│  │ • match_pot()   │    │                 │    │ • /tutorial/*   │         │
│  └────────┬────────┘    └────────┬────────┘    └─────────────────┘         │
│           │                      │                                          │
│           ▼                      ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────┐       │
│  │                         POTs Library                             │       │
│  ├─────────────────────────────────────────────────────────────────┤       │
│  │                                                                  │       │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │       │
│  │  │   REPAIR     │  │ MAINTENANCE  │  │   INCIDENT   │          │       │
│  │  │              │  │              │  │              │          │       │
│  │  │ • camera_    │  │ • daily      │  │ • triage     │          │       │
│  │  │   repair     │  │ • weekly     │  │ • response   │          │       │
│  │  │ • services_  │  │              │  │              │          │       │
│  │  │   repair     │  │              │  │              │          │       │
│  │  │ • api_repair │  │              │  │              │          │       │
│  │  └──────────────┘  └──────────────┘  └──────────────┘          │       │
│  │                                                                  │       │
│  │  ┌──────────────┐                                               │       │
│  │  │  DIAGNOSTIC  │                                               │       │
│  │  │              │                                               │       │
│  │  │ • full       │                                               │       │
│  │  └──────────────┘                                               │       │
│  └─────────────────────────────────────────────────────────────────┘       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Estructura de Directorios

```
modules/humanoid/quality/
├── __init__.py           # Exports principales
├── models.py             # Modelos de datos (POT, POTStep, etc)
├── registry.py           # Búsqueda y selección de POTs
├── executor.py           # Motor de ejecución
├── api.py                # Endpoints REST
├── pots/                 # Biblioteca de POTs
│   ├── __init__.py       # Registry de POTs
│   ├── camera_repair.py
│   ├── services_repair.py
│   ├── api_repair.py
│   ├── maintenance_daily.py
│   ├── maintenance_weekly.py
│   ├── incident_triage.py
│   ├── incident_response.py
│   └── diagnostic_full.py
├── snapshots/            # Estados capturados durante ejecución
└── reports/              # Reportes de ejecución de POTs
```

## POTs Disponibles

### Categoría: REPAIR (Reparaciones)

| POT ID | Nombre | Severidad | Descripción |
|--------|--------|-----------|-------------|
| `camera_repair` | Reparación de Cámaras | MEDIUM | Diagnóstico y reparación de cámaras USB/integradas |
| `services_repair` | Reparación de Servicios | HIGH | Restauración de servicios NEXUS, Robot, Push |
| `api_repair` | Reparación de APIs | MEDIUM | Diagnóstico y fix de endpoints degradados |

### Categoría: MAINTENANCE (Mantenimiento)

| POT ID | Nombre | Severidad | Descripción |
|--------|--------|-----------|-------------|
| `maintenance_daily` | Mantenimiento Diario | LOW | Limpieza de logs, cache, verificación de espacio |
| `maintenance_weekly` | Mantenimiento Semanal | MEDIUM | VACUUM de DBs, revisión de dependencias, reinicio preventivo |

### Categoría: INCIDENT (Incidentes)

| POT ID | Nombre | Severidad | Descripción |
|--------|--------|-----------|-------------|
| `incident_triage` | Triaje de Incidentes | LOW | Clasificación y priorización de incidentes |
| `incident_response` | Respuesta a Incidentes | HIGH | Ciclo completo de respuesta (CDIRV) |

### Categoría: DIAGNOSTIC (Diagnósticos)

| POT ID | Nombre | Severidad | Descripción |
|--------|--------|-----------|-------------|
| `diagnostic_full` | Diagnóstico Completo | LOW | Evaluación exhaustiva de todos los componentes |

## Estructura de un POT

```python
POT(
    id="camera_repair",
    name="Reparación de Cámaras",
    description="...",
    category=POTCategory.REPAIR,
    severity=POTSeverity.MEDIUM,
    
    # Triggers (cuándo activar este POT)
    trigger_check_ids=["camera_*", "vision_*"],
    trigger_keywords=["camera", "webcam", "opencv"],
    
    # Requisitos
    prerequisites=["Robot backend corriendo", ...],
    required_services=["robot", "push"],
    required_permissions=["camera_control"],
    
    # Objetivos
    objectives=["Identificar cámaras", "Diagnosticar fallas", ...],
    success_criteria="Al menos una cámara captura frames",
    estimated_duration_minutes=5,
    
    # Tutorial
    tutorial_overview="...",
    best_practices=["Usar subprocess aislado", ...],
    warnings=["NO abrir múltiples cámaras", ...],
    
    # Pasos
    steps=[
        POTStep(
            id="check_services",
            name="Verificar servicios",
            step_type=StepType.HTTP,
            http_url="http://127.0.0.1:8002/api/health",
            tutorial_notes="Antes de tocar cámaras, verificar...",
            ...
        ),
        ...
    ],
    
    # Rollback (opcional)
    has_rollback=True,
    rollback_steps=[...],
)
```

## Tipos de Pasos

| Tipo | Descripción | Campos Usados |
|------|-------------|---------------|
| `COMMAND` | Ejecutar comando shell | `command` |
| `SCRIPT` | Ejecutar script Python | `script_path`, `script_function` |
| `HTTP` | Llamada HTTP REST | `http_method`, `http_url`, `http_body` |
| `CHECK` | Verificar condición | `check_expression` |
| `WAIT` | Esperar tiempo | `wait_seconds` |
| `LOG` | Solo logging | - |
| `NOTIFY` | Notificar (Telegram) | `notify_channel`, `notify_message` |
| `SNAPSHOT` | Capturar estado | - |
| `CONFIRM` | Requiere confirmación | - |
| `ROLLBACK` | Paso de rollback | - |

## API Endpoints

### Listar POTs
```http
GET /quality/pots?category=repair&severity=medium
```

### Obtener detalles de POT
```http
GET /quality/pots/{pot_id}
```

### Ejecutar POT
```http
POST /quality/pots/execute
{
    "pot_id": "camera_repair",
    "context": {"incident_id": "abc123"},
    "dry_run": false
}
```

### Obtener tutorial
```http
GET /quality/tutorial/{pot_id}
```

### Buscar POT para incidente
```http
GET /quality/pots/match?check_id=camera_health&message=frame_error
```

### Listar reportes
```http
GET /quality/reports?limit=20
```

## Integración con Workshop Central

El Workshop Central usa automáticamente los POTs para ejecutar reparaciones:

```python
# En atlas_central_workshop.py
def _select_runbook(check_id, message):
    # Usa el sistema de POTs
    from modules.humanoid.quality import get_pot_by_incident
    pot = get_pot_by_incident(check_id=check_id, message=message)
    return pot.id if pot else "generic_repair"

def _execute_runbook(runbook_id):
    # Ejecuta el POT
    from modules.humanoid.quality import get_pot, execute_pot
    pot = get_pot(runbook_id)
    result = execute_pot(pot)
    return result.step_results
```

## Flujo de Ejecución

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Incidente  │────▶│    Triaje    │────▶│ Seleccionar  │
│   Detectado  │     │   (POT)      │     │     POT      │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                  │
         ┌────────────────────────────────────────┘
         │
         ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Verificar  │────▶│   Ejecutar   │────▶│  Verificar   │
│ Prerequisites│     │    Pasos     │     │   Éxito      │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                  │
         ┌──────────────┬─────────────────────────┘
         │              │
         ▼              ▼
┌──────────────┐  ┌──────────────┐
│   Generar    │  │   Rollback   │
│   Reporte    │  │  (si falla)  │
└──────────────┘  └──────────────┘
```

## Crear Nuevo POT

1. Crear archivo en `modules/humanoid/quality/pots/mi_nuevo_pot.py`:

```python
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType

def get_pot() -> POT:
    return POT(
        id="mi_nuevo_pot",
        name="Mi Nuevo Procedimiento",
        category=POTCategory.REPAIR,
        severity=POTSeverity.MEDIUM,
        # ... resto de campos
        steps=[
            POTStep(
                id="paso_1",
                name="Primer paso",
                step_type=StepType.COMMAND,
                command="echo 'Hola'",
            ),
        ],
    )
```

2. Registrar en `modules/humanoid/quality/pots/__init__.py`:

```python
from . import mi_nuevo_pot

_POT_MODULES: Dict[str, Callable] = {
    # ... existentes
    "mi_nuevo_pot": mi_nuevo_pot.get_pot,
}
```

## Estados de Reporte

Cada ejecución de POT genera un reporte estructurado:

```json
{
  "pot_id": "camera_repair",
  "pot_name": "Reparación de Cámaras",
  "ok": true,
  "started_at": "2026-02-16T05:30:00Z",
  "ended_at": "2026-02-16T05:35:00Z",
  "elapsed_ms": 300000,
  "steps_total": 8,
  "steps_ok": 7,
  "steps_failed": 0,
  "steps_skipped": 1,
  "step_results": [
    {
      "step_id": "check_services",
      "step_name": "Verificar servicios",
      "ok": true,
      "elapsed_ms": 150,
      "output": "{\"status\": \"healthy\"}"
    },
    // ...
  ],
  "verification_ok": true,
  "rollback_executed": false,
  "report_path": "modules/humanoid/quality/reports/pot_report_camera_repair_20260216_053500.json"
}
```

## Variables de Entorno

| Variable | Default | Descripción |
|----------|---------|-------------|
| `QUALITY_POT_TIMEOUT_DEFAULT` | `60` | Timeout por defecto para pasos |
| `QUALITY_POT_DRY_RUN` | `false` | Modo simulación global |
| `QUALITY_NOTIFY_ENABLED` | `true` | Habilitar notificaciones |

## Mejores Prácticas

1. **Pasos Atómicos**: Cada paso debe hacer una sola cosa
2. **Capturas de Output**: Usar `capture_output=True` para debugging
3. **Rollback**: Siempre definir pasos de rollback para operaciones riesgosas
4. **Tutorial Notes**: Documentar el "por qué" de cada paso
5. **Timeouts**: Ajustar timeouts según la operación real
6. **continue_on_failure**: Usar solo cuando el paso es opcional
