"""
POT: Full Diagnostic (Diagnóstico Completo)
============================================
Procedimiento de diagnóstico exhaustivo del sistema.

Triggers:
- Cuando se necesita evaluación completa del sistema
- Post-incidente para verificar salud general
- Antes de actualizaciones mayores

Severidad: LOW (solo lectura, no modifica nada)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="diagnostic_full",
        name="Diagnóstico Completo",
        description="""
Diagnóstico exhaustivo de todos los componentes del sistema.
Solo lectura: no realiza cambios, solo recopila información.
Ideal para evaluar estado general o preparar actualizaciones.
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["diagnostic_*", "health_check_full"],
        trigger_keywords=["diagnostic", "diagnóstico", "full", "complete", "status"],
        
        prerequisites=[
            "Sistema accesible (al menos Push dashboard)",
        ],
        required_services=["push"],
        
        objectives=[
            "Verificar estado de todos los servicios",
            "Medir tiempos de respuesta",
            "Verificar conectividad entre componentes",
            "Detectar anomalías en recursos (disco, memoria)",
            "Generar reporte completo de estado",
        ],
        success_criteria="Reporte generado con todos los componentes evaluados",
        estimated_duration_minutes=3,
        
        tutorial_overview="""
## Guía de Diagnóstico Completo

### Componentes Evaluados
1. **Servicios Core**
   - Push Dashboard (8791)
   - Robot Backend (8002)
   - NEXUS (8000)

2. **Subsistemas**
   - Cámaras / Visión
   - ANS (Sistema Nervioso)
   - Scheduler
   - Base de datos

3. **Recursos**
   - Espacio en disco
   - Uso de memoria
   - Conectividad de red

### Métricas Capturadas
- Tiempo de respuesta de cada endpoint
- Health score
- Uptime de servicios
- Incidentes abiertos
- Jobs pendientes del scheduler
        """.strip(),
        
        best_practices=[
            "Ejecutar diagnóstico completo periódicamente (diario)",
            "Comparar con diagnósticos anteriores para detectar degradación",
            "Guardar resultados para análisis de tendencias",
            "No ejecutar durante operaciones de carga pesada",
        ],
        
        related_pots=["maintenance_daily", "incident_triage"],
        tags=["diagnostic", "health", "status", "report", "readonly"],
        
        steps=[
            POTStep(
                id="start_diagnostic",
                name="Iniciar diagnóstico",
                description="Registrar inicio del diagnóstico",
                step_type=StepType.LOG,
                tutorial_notes="Marca el inicio del diagnóstico para medir duración total.",
            ),
            
            # Servicios Core
            POTStep(
                id="check_push",
                name="Verificar Push Dashboard",
                description="Health check del dashboard principal",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_robot",
                name="Verificar Robot Backend",
                description="Health check del backend de hardware",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8002/api/health",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_nexus",
                name="Verificar NEXUS",
                description="Health check del coordinador",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8000/health",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # Subsistemas
            POTStep(
                id="check_cameras",
                name="Verificar cámaras",
                description="Estado del sistema de visión",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/cuerpo/api/camera/status",
                timeout_seconds=20,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_ans",
                name="Verificar ANS",
                description="Estado del Sistema Nervioso Autónomo",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/status",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_scheduler",
                name="Verificar Scheduler",
                description="Estado de jobs programados",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/scheduler/jobs",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_incidents",
                name="Verificar incidentes",
                description="Listar incidentes abiertos",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open&limit=20",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_approvals",
                name="Verificar aprobaciones pendientes",
                description="Listar aprobaciones esperando",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/approvals/list?status=pending",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # Recursos del sistema
            POTStep(
                id="check_disk",
                name="Verificar espacio en disco",
                description="Espacio libre en unidad C:",
                step_type=StepType.COMMAND,
                command='powershell -Command "$d=Get-PSDrive C; @{Used=$d.Used; Free=$d.Free; PctFree=[math]::Round($d.Free/($d.Used+$d.Free)*100,2)} | ConvertTo-Json"',
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_memory",
                name="Verificar memoria",
                description="Uso de memoria del sistema",
                step_type=StepType.COMMAND,
                command='powershell -Command "$os=Get-CimInstance Win32_OperatingSystem; @{TotalMB=[math]::Round($os.TotalVisibleMemorySize/1024); FreeMB=[math]::Round($os.FreePhysicalMemory/1024); PctFree=[math]::Round($os.FreePhysicalMemory/$os.TotalVisibleMemorySize*100,2)} | ConvertTo-Json"',
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_python_processes",
                name="Contar procesos Python",
                description="Número de procesos Python activos",
                step_type=StepType.COMMAND,
                command='powershell -Command "(Get-Process -Name python* -ErrorAction SilentlyContinue).Count"',
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # Workshop
            POTStep(
                id="check_workshop",
                name="Verificar Workshop",
                description="Estado del Taller Central",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/workshop/status",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # Generar reporte
            POTStep(
                id="generate_report",
                name="Generar reporte",
                description="Consolidar resultados en reporte",
                step_type=StepType.LOG,
                tutorial_notes="""
El motor de POTs consolida todos los outputs capturados en un reporte JSON:
{
  "timestamp": "...",
  "duration_ms": ...,
  "services": {...},
  "subsystems": {...},
  "resources": {...},
  "incidents_open": ...,
  "overall_status": "healthy|degraded|critical"
}
                """,
            ),
            
            POTStep(
                id="log_diagnostic",
                name="Registrar diagnóstico",
                description="Log en bitácora",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[DIAGNÓSTICO] Diagnóstico completo ejecutado por POT",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
