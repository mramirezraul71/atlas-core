"""
POT: Full Deployment (Despliegue Completo)
==========================================
Procedimiento de despliegue completo a producción.

Triggers:
- Cuando hay versión lista para deploy
- Solicitud manual de deployment

Severidad: CRITICAL (afecta producción)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="deployment_full",
        name="Despliegue Completo",
        description="""
Procedimiento completo de despliegue que incluye:
- Verificación pre-deploy
- Backup del estado actual
- Commit y push de cambios
- Actualización de servicios
- Verificación post-deploy
- Notificación a todos los canales
        """.strip(),
        category=POTCategory.DEPLOYMENT,
        severity=POTSeverity.CRITICAL,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["deploy_*", "release_*"],
        trigger_keywords=["deploy", "deployment", "release", "produccion", "production"],
        
        prerequisites=[
            "Todos los tests pasando",
            "Cambios revisados y aprobados",
            "Backup reciente disponible",
            "Ventana de mantenimiento si necesario",
        ],
        required_services=["push", "robot"],
        required_permissions=["deploy", "git_push", "service_restart", "notify_all"],
        
        objectives=[
            "Verificar estado pre-deploy",
            "Crear backup/snapshot",
            "Commit y push de cambios",
            "Reiniciar servicios con nuevo código",
            "Verificar funcionamiento",
            "Notificar a todos los stakeholders",
        ],
        success_criteria="Sistema en producción con nuevo código, todos los health checks pasando",
        estimated_duration_minutes=15,
        
        tutorial_overview="""
## Guía de Despliegue Completo

### Pipeline de Deployment
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│Pre-check │────▶│  Backup  │────▶│  Deploy  │────▶│  Verify  │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
     │                                                   │
     │                                                   │
     ▼                                                   ▼
┌──────────┐                                       ┌──────────┐
│  Abort   │                                       │  Notify  │
│ if fail  │                                       │   All    │
└──────────┘                                       └──────────┘
```

### Checklist Pre-Deploy
- [ ] Tests unitarios pasando
- [ ] Tests de integración pasando
- [ ] Código revisado
- [ ] Documentación actualizada
- [ ] Changelog actualizado
- [ ] Backup verificado

### Canales de Notificación
1. **Telegram**: Owner/Admin
2. **OPS Bus**: Sistema interno
3. **Bitácora ANS**: Registro permanente
4. **Dashboard**: Actualización visual
        """.strip(),
        
        best_practices=[
            "SIEMPRE crear backup antes de deploy",
            "Deploy en ventana de bajo tráfico",
            "Tener plan de rollback listo",
            "Monitorear métricas post-deploy",
            "Notificar a todos los stakeholders",
        ],
        
        warnings=[
            "CRITICAL: Requiere aprobación para ejecutar",
            "Tener acceso a rollback en caso de fallo",
            "No hacer deploy en viernes tarde",
            "Verificar que no hay operaciones críticas en curso",
        ],
        
        related_pots=["git_commit", "git_push", "services_repair", "diagnostic_full"],
        tags=["deploy", "deployment", "production", "release", "critical"],
        has_rollback=True,
        
        steps=[
            # Pre-checks
            POTStep(
                id="precheck_health",
                name="Verificar salud actual",
                description="Confirmar que el sistema está estable antes de deploy",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=15,
                check_expression="response.get('score', 0) >= 80",
                tutorial_notes="Solo proceder si health score >= 80",
            ),
            
            POTStep(
                id="precheck_no_incidents",
                name="Verificar sin incidentes críticos",
                description="Confirmar que no hay incidentes abiertos",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open",
                timeout_seconds=10,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="notify_deploy_start",
                name="Notificar inicio de deploy",
                description="Avisar que comienza el deployment",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🚀 Iniciando deployment completo...",
            ),
            
            # Backup
            POTStep(
                id="create_backup_snapshot",
                name="Crear snapshot de backup",
                description="Guardar estado actual para rollback",
                step_type=StepType.COMMAND,
                command="git stash push -m 'DEPLOY_BACKUP' --include-untracked",
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="tag_pre_deploy",
                name="Crear tag pre-deploy",
                description="Marcar punto de rollback",
                step_type=StepType.COMMAND,
                command="git tag -f pre-deploy-backup",
                timeout_seconds=10,
            ),
            
            # Deploy
            POTStep(
                id="stage_all_changes",
                name="Stage todos los cambios",
                description="Agregar cambios al staging",
                step_type=StepType.COMMAND,
                command="git add -A",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="commit_deploy",
                name="Commit de deploy",
                description="Crear commit con los cambios",
                step_type=StepType.COMMAND,
                command='git commit -m "deploy: automated deployment by ATLAS" --allow-empty',
                timeout_seconds=30,
                capture_output=True,
            ),
            
            POTStep(
                id="push_to_remote",
                name="Push al remoto",
                description="Enviar cambios al repositorio",
                step_type=StepType.COMMAND,
                command="git push origin HEAD",
                timeout_seconds=180,
                capture_output=True,
            ),
            
            POTStep(
                id="restart_all_services",
                name="Reiniciar todos los servicios",
                description="Aplicar nuevo código reiniciando servicios",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service all",
                timeout_seconds=180,
            ),
            
            POTStep(
                id="wait_services",
                name="Esperar servicios",
                description="Dar tiempo a los servicios para iniciar",
                step_type=StepType.WAIT,
                wait_seconds=15,
            ),
            
            # Verification
            POTStep(
                id="verify_health",
                name="Verificar salud post-deploy",
                description="Confirmar que el sistema está funcionando",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=30,
                retries=3,
                retry_delay_seconds=5,
                check_expression="response.get('score', 0) >= 70",
            ),
            
            POTStep(
                id="verify_robot",
                name="Verificar Robot backend",
                description="Confirmar que Robot responde",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8002/api/health",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="run_smoke_test",
                name="Ejecutar smoke test",
                description="Pruebas básicas de funcionalidad",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_nexus_ports.py",
                timeout_seconds=60,
                capture_output=True,
            ),
            
            # Notify all channels
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log permanente del deployment",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[DEPLOY] Deployment completo exitoso por POT deployment_full",
                    "ok": True,
                    "source": "quality_pot"
                },
            ),
            
            POTStep(
                id="notify_telegram",
                name="Notificar a Telegram",
                description="Avisar al owner del éxito",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="✅ Deployment completado exitosamente. Sistema operativo.",
            ),
            
            POTStep(
                id="notify_ops",
                name="Notificar a OPS Bus",
                description="Registrar en canal OPS interno",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="Deployment completado. Todos los servicios verificados.",
                continue_on_failure=True,
            ),
            
            POTStep(
                id="update_dashboard_status",
                name="Actualizar estado en Dashboard",
                description="Refrescar información del dashboard",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/status",
                timeout_seconds=15,
                continue_on_failure=True,
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_notify_start",
                name="Notificar inicio de rollback",
                description="Avisar que se está revirtiendo",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="⚠️ Deployment fallido. Iniciando ROLLBACK...",
            ),
            
            POTStep(
                id="rollback_reset",
                name="Revertir a tag pre-deploy",
                description="Volver al estado anterior al deploy",
                step_type=StepType.COMMAND,
                command="git reset --hard pre-deploy-backup",
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_pop_stash",
                name="Restaurar stash si existe",
                description="Recuperar cambios guardados",
                step_type=StepType.COMMAND,
                command="git stash pop",
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="rollback_restart",
                name="Reiniciar servicios",
                description="Reiniciar con código anterior",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service all",
                timeout_seconds=180,
            ),
            
            POTStep(
                id="rollback_verify",
                name="Verificar rollback",
                description="Confirmar que el sistema funciona",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_notify_complete",
                name="Notificar rollback completado",
                description="Avisar resultado del rollback",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🔄 ROLLBACK completado. Sistema restaurado al estado anterior.",
            ),
        ],
    )
