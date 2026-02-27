"""
POT: Services Repair (Reparación de Servicios)
==============================================
Procedimiento para diagnosticar y reparar servicios caídos o degradados.

Triggers:
- check_id: nexus_*, robot_*, gateway_*, service_*
- keywords: service, nexus, robot, gateway, port, connection

Severidad: HIGH (servicios caídos afectan funcionalidad core)
"""
import os
import sys
from pathlib import Path
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType

# Ruta absoluta al repo para que los comandos funcionen desde cualquier cwd
_REPO_ROOT = str(Path(__file__).resolve().parent.parent.parent.parent.parent)
_SCRIPTS_DIR = os.path.join(_REPO_ROOT, "scripts")
_PS1 = os.path.join(_SCRIPTS_DIR, "restart_service_clean.ps1")
_START_SERVICES_PY = os.path.join(_SCRIPTS_DIR, "start_nexus_services.py")
_PYTHON = sys.executable


def get_pot() -> POT:
    return POT(
        id="services_repair",
        name="Reparación de Servicios",
        description="""
Procedimiento para diagnosticar y restaurar servicios críticos de ATLAS:
- NEXUS (puerto 8000): Coordinador central
- Robot Backend (puerto 8002): Control de hardware
- Push Dashboard (puerto 8791): Interfaz de usuario
        """.strip(),
        category=POTCategory.REPAIR,
        severity=POTSeverity.HIGH,
        version="2.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["nexus_health", "robot_health", "gateway_health", "service_*", "port_*"],
        trigger_keywords=["service", "nexus", "robot", "gateway", "port", "connection", "refused", "timeout"],
        
        prerequisites=[
            "Acceso a PowerShell con permisos de administrador",
            "Scripts de reinicio disponibles en scripts/",
            "Puertos 8000, 8002, 8791 no bloqueados por firewall",
        ],
        required_services=[],  # No requiere servicios previos (los va a reparar)
        required_permissions=["service_restart", "process_kill"],
        
        objectives=[
            "Identificar qué servicio(s) está(n) caído(s)",
            "Limpiar procesos zombie si existen",
            "Reiniciar servicios en orden correcto",
            "Verificar conectividad entre servicios",
        ],
        success_criteria="Todos los servicios responden a /health con status 200",
        estimated_duration_minutes=3,
        
        tutorial_overview="""
## Guía de Reparación de Servicios

### Arquitectura de Servicios ATLAS
```
                    ┌─────────────────┐
                    │  Push Dashboard │
                    │   (port 8791)   │
                    └────────┬────────┘
                             │ proxy
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│     NEXUS      │  │  Robot Backend │  │  Otros APIs    │
│  (port 8000)   │  │   (port 8002)  │  │                │
└────────────────┘  └────────────────┘  └────────────────┘
```

### Orden de Reinicio
1. **Robot Backend** primero (hardware layer)
2. **NEXUS** segundo (coordinación)
3. **Push Dashboard** último (presentation layer)

### Causas Comunes de Fallo
- Proceso zombie ocupando puerto
- Crash por excepción no manejada
- Timeout de base de datos
- Memoria agotada
        """.strip(),
        
        best_practices=[
            "Siempre verificar estado ANTES de reiniciar",
            "Matar procesos zombie antes de iniciar nuevos",
            "Esperar a que el servicio esté completamente UP antes del siguiente",
            "Verificar logs si el reinicio falla repetidamente",
        ],
        
        warnings=[
            "Reiniciar servicios puede interrumpir operaciones en curso",
            "Si un servicio falla repetidamente, revisar logs antes de reintentar",
            "No reiniciar todos a la vez, hacerlo secuencialmente",
        ],
        
        related_pots=["diagnostic_full", "recovery_full", "port_repair"],
        tags=["services", "nexus", "robot", "gateway", "repair", "restart"],
        has_rollback=False,  # Servicios no tienen rollback tradicional
        
        steps=[
            # Diagnóstico inicial
            POTStep(
                id="diagnose_robot",
                name="Diagnosticar Robot Backend",
                description="Verificar si el Robot backend responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8002/api/health",
                timeout_seconds=8,
                continue_on_failure=True,
                capture_output=True,
                tutorial_notes="""
Primer paso: verificar qué servicios están caídos.
Si Robot responde 200, está OK. Si falla, necesita reinicio.
                """,
            ),
            
            POTStep(
                id="diagnose_nexus",
                name="Diagnosticar NEXUS",
                description="Verificar si NEXUS responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8000/health",
                timeout_seconds=8,
                continue_on_failure=True,
                capture_output=True,
            ),
            
            POTStep(
                id="diagnose_push",
                name="Diagnosticar Push Dashboard",
                description="Verificar si Push Dashboard responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=8,
                continue_on_failure=True,
                capture_output=True,
            ),
            
            # Limpieza de procesos zombie — SOLO si robot NO responde
            POTStep(
                id="kill_zombie_processes",
                name="Limpiar procesos zombie (solo si robot caído)",
                description="Matar procesos que ocupen puertos solo cuando no responden",
                step_type=StepType.COMMAND,
                command=(
                    'powershell -Command "'
                    'try { $r = Invoke-WebRequest -Uri http://127.0.0.1:8002/api/health -TimeoutSec 3 -UseBasicParsing -EA Stop; exit 0 } catch {};'
                    'Get-NetTCPConnection -LocalPort 8000,8002 -State Listen -ErrorAction SilentlyContinue'
                    ' | ForEach-Object { Stop-Process -Id $_.OwningProcess -Force -ErrorAction SilentlyContinue }"'
                ),
                timeout_seconds=15,
                continue_on_failure=True,
                tutorial_notes="""
Primero verifica si robot responde. Si responde, no mata nada.
Si no responde, limpia procesos zombie para liberar puertos.
                """,
            ),
            
            POTStep(
                id="wait_port_release",
                name="Esperar liberación de puertos",
                description="Dar tiempo al SO para liberar puertos",
                step_type=StepType.WAIT,
                wait_seconds=3,
            ),
            
            # Reinicio secuencial
            POTStep(
                id="restart_robot",
                name="Reiniciar Robot Backend",
                description="Iniciar Robot backend (hardware layer)",
                step_type=StepType.COMMAND,
                command=f'"{_PYTHON}" "{_START_SERVICES_PY}" --robot-only',
                timeout_seconds=60,
                continue_on_failure=True,
                capture_output=True,
                tutorial_notes="""
Robot Backend se inicia primero porque:
1. Controla hardware (cámaras, sensores)
2. Otros servicios dependen de él
3. Es más rápido de iniciar que NEXUS
                """,
            ),
            
            POTStep(
                id="wait_robot_ready",
                name="Esperar Robot listo",
                description="Dar tiempo al Robot para inicializar completamente",
                step_type=StepType.WAIT,
                wait_seconds=5,
            ),
            
            POTStep(
                id="verify_robot",
                name="Verificar Robot activo",
                description="Confirmar que Robot responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8002/api/health",
                timeout_seconds=15,
                retries=3,
                retry_delay_seconds=3,
            ),
            
            POTStep(
                id="restart_nexus",
                name="Reiniciar NEXUS",
                description="Iniciar NEXUS (coordinación central)",
                step_type=StepType.COMMAND,
                command=f'powershell -ExecutionPolicy Bypass -File "{_PS1}" -Service nexus',
                timeout_seconds=90,
                continue_on_failure=True,
                capture_output=True,
                condition="context.get('diagnose_nexus_failed', False)",
                tutorial_notes="""
NEXUS solo se reinicia si estaba caído.
Es el servicio más pesado (carga modelos, etc).
                """,
            ),
            
            POTStep(
                id="restart_push",
                name="Reiniciar Push Dashboard",
                description="Reiniciar el dashboard si estaba caído",
                step_type=StepType.COMMAND,
                command=f'powershell -ExecutionPolicy Bypass -File "{_PS1}" -Service push',
                timeout_seconds=60,
                continue_on_failure=True,
                condition="context.get('diagnose_push_failed', False)",
            ),
            
            # Verificación final
            POTStep(
                id="final_health_check",
                name="Verificación final de salud",
                description="Ejecutar script de diagnóstico completo",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_nexus_ports.py",
                timeout_seconds=30,
                capture_output=True,
                tutorial_notes="""
El script check_nexus_ports.py verifica:
- Conectividad a cada puerto
- Respuesta de /health
- Muestra resumen de estado
                """,
            ),
            
            POTStep(
                id="log_completion",
                name="Registrar en bitácora",
                description="Log de reparación completada",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[REPARACIÓN] Servicios reparados por POT services_repair",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
