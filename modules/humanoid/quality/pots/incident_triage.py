"""
POT: Incident Triage (Triaje de Incidentes)
===========================================
Procedimiento para clasificar y priorizar incidentes nuevos.

Triggers:
- Cuando llega un nuevo incidente al Workshop
- Cuando hay múltiples incidentes pendientes

Severidad: LOW (es un proceso de clasificación, no de reparación)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="incident_triage",
        name="Triaje de Incidentes",
        description="""
Procedimiento para clasificar, priorizar y asignar incidentes nuevos.
Determina la severidad real, el POT apropiado y si requiere aprobación.
        """.strip(),
        category=POTCategory.INCIDENT,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["incident_new", "triage_*"],
        trigger_keywords=["triage", "classify", "prioritize", "incident"],
        
        prerequisites=[
            "Acceso al Workshop Central",
            "Acceso al ANS para consultar incidentes",
        ],
        required_services=["push"],
        
        objectives=[
            "Clasificar incidentes por severidad (low, medium, high, critical)",
            "Identificar el POT apropiado para cada incidente",
            "Determinar si requiere aprobación manual",
            "Priorizar orden de ejecución",
        ],
        success_criteria="Todos los incidentes clasificados con POT asignado",
        estimated_duration_minutes=2,
        
        tutorial_overview="""
## Guía de Triaje de Incidentes

### Matriz de Severidad
| Criterio | LOW | MEDIUM | HIGH | CRITICAL |
|----------|-----|--------|------|----------|
| Afecta usuarios | No | Parcial | Sí | Todos |
| Servicio caído | No | Degradado | 1 servicio | Múltiples |
| Datos en riesgo | No | No | Posible | Sí |
| Auto-reparable | Sí | Sí | Posible | No |

### Reglas de Asignación de POT
```
check_id contiene "camera" → camera_repair
check_id contiene "nexus/robot/gateway" → services_repair
check_id contiene "api/health" → api_repair
check_id contiene "deps/llm" → dependency_repair
check_id contiene "disk/storage" → disk_cleanup
default → generic_repair
```

### Priorización
1. CRITICAL: Inmediato, notificar owner
2. HIGH: Siguiente en cola, puede requerir aprobación
3. MEDIUM: Cola normal
4. LOW: Puede esperar, agregar al batch
        """.strip(),
        
        best_practices=[
            "Revisar incidentes similares recientes antes de clasificar",
            "Agrupar incidentes relacionados para tratarlos juntos",
            "Escalar a CRITICAL si hay múltiples incidentes HIGH",
            "Documentar razón de clasificación para auditoría",
        ],
        
        related_pots=["incident_response", "incident_postmortem"],
        tags=["incident", "triage", "classification", "priority"],
        
        steps=[
            POTStep(
                id="fetch_open_incidents",
                name="Obtener incidentes abiertos",
                description="Consultar incidentes pendientes del ANS",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open&limit=50",
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="Obtenemos todos los incidentes abiertos para clasificar.",
            ),
            
            POTStep(
                id="classify_by_check_id",
                name="Clasificar por check_id",
                description="Asignar POT basado en el check_id del incidente",
                step_type=StepType.SCRIPT,
                script_function="classify_incidents",
                timeout_seconds=10,
                tutorial_notes="""
La clasificación usa estas reglas:
- camera_* → camera_repair (MEDIUM)
- nexus_*/robot_*/gateway_* → services_repair (HIGH)
- api_*/health_* → api_repair (MEDIUM)
- deps_* → dependency_repair (LOW)
- disk_* → disk_cleanup (LOW)
                """,
            ),
            
            POTStep(
                id="check_related_incidents",
                name="Detectar incidentes relacionados",
                description="Buscar patrones de fallo conjunto",
                step_type=StepType.CHECK,
                check_expression="len([i for i in incidents if i.get('check_id', '').startswith('service')]) > 1",
                tutorial_notes="""
Si hay múltiples servicios fallando, puede ser un problema upstream:
- Red caída
- Base de datos no disponible
- Recursos agotados (memoria/disco)
                """,
            ),
            
            POTStep(
                id="prioritize_queue",
                name="Priorizar cola de trabajo",
                description="Ordenar incidentes por severidad",
                step_type=StepType.LOG,
                tutorial_notes="""
Orden de prioridad:
1. CRITICAL → Ejecutar inmediatamente
2. HIGH → Siguiente en cola
3. MEDIUM → Cola normal
4. LOW → Agregar al batch de mantenimiento
                """,
            ),
            
            POTStep(
                id="log_triage_result",
                name="Registrar resultado de triaje",
                description="Guardar clasificación en bitácora",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[TRIAJE] Incidentes clasificados y priorizados",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
