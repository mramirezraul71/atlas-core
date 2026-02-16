"""
POT: API Repair (Reparación de APIs)
====================================
Procedimiento para diagnosticar y reparar endpoints API degradados.

Triggers:
- check_id: api_*, endpoint_*, health_*
- keywords: api, endpoint, 500, 502, 503, timeout, response

Severidad: MEDIUM
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="api_repair",
        name="Reparación de APIs",
        description="""
Procedimiento para diagnosticar y reparar endpoints API que devuelven errores
o responden lentamente. Incluye verificación de rutas, reinicio selectivo y
pruebas de carga básicas.
        """.strip(),
        category=POTCategory.REPAIR,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["api_health", "endpoint_*", "http_*"],
        trigger_keywords=["api", "endpoint", "500", "502", "503", "504", "timeout", "error", "response"],
        
        prerequisites=[
            "Dashboard Push accesible",
            "Acceso a logs del servidor",
        ],
        required_services=["push"],
        
        objectives=[
            "Identificar endpoint(s) fallando",
            "Diagnosticar causa raíz (timeout, exception, dependencia)",
            "Aplicar corrección apropiada",
            "Verificar respuesta correcta",
        ],
        success_criteria="Endpoint(s) afectados responden 200 con payload válido",
        estimated_duration_minutes=2,
        
        tutorial_overview="""
## Guía de Reparación de APIs

### Códigos de Error Comunes
| Código | Significado | Acción |
|--------|-------------|--------|
| 500 | Internal Server Error | Revisar logs, reiniciar servicio |
| 502 | Bad Gateway | Proxy no puede conectar al backend |
| 503 | Service Unavailable | Servicio sobrecargado o en mantenimiento |
| 504 | Gateway Timeout | Backend no responde a tiempo |
| 422 | Unprocessable Entity | Payload inválido |

### Flujo de Diagnóstico
1. Verificar que el servicio base esté corriendo
2. Revisar logs recientes para excepciones
3. Probar endpoint con payload mínimo
4. Si falla, reiniciar servicio afectado
        """.strip(),
        
        best_practices=[
            "Siempre revisar logs antes de reiniciar",
            "Probar con payload mínimo primero",
            "Verificar que dependencias (DB, Redis) estén UP",
            "Documentar el error para prevenir recurrencia",
        ],
        
        related_pots=["services_repair", "diagnostic_full"],
        tags=["api", "http", "endpoint", "repair"],
        
        steps=[
            POTStep(
                id="check_base_health",
                name="Verificar salud base",
                description="Confirmar que /health responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=10,
                tutorial_notes="Si /health falla, el problema es más grave que un endpoint específico.",
            ),
            
            POTStep(
                id="trigger_ans_cycle",
                name="Ejecutar ciclo ANS",
                description="Forzar revisión del Sistema Nervioso",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/run-now",
                http_body={"mode": "auto"},
                timeout_seconds=60,
                continue_on_failure=True,
                tutorial_notes="""
El ANS puede detectar y auto-reparar muchos problemas.
Lo ejecutamos primero para aprovechar los heals existentes.
                """,
            ),
            
            POTStep(
                id="wait_ans",
                name="Esperar ciclo ANS",
                description="Dar tiempo al ANS para ejecutar heals",
                step_type=StepType.WAIT,
                wait_seconds=5,
            ),
            
            POTStep(
                id="check_incidents",
                name="Verificar incidentes recientes",
                description="Ver si hay incidentes relacionados",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open&limit=5",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="restart_push_if_needed",
                name="Reiniciar Push si necesario",
                description="Reiniciar el servicio Push para limpiar estado",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service push",
                timeout_seconds=90,
                condition="context.get('restart_required', False)",
                tutorial_notes="Solo reiniciar si los pasos anteriores no resolvieron.",
            ),
            
            POTStep(
                id="verify_endpoints",
                name="Verificar endpoints críticos",
                description="Probar endpoints principales",
                step_type=StepType.SCRIPT,
                script_path="scripts/smoke_test_endpoints.py",
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="log_result",
                name="Registrar resultado",
                description="Log en bitácora",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[REPARACIÓN] APIs verificadas por POT api_repair",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
