"""
POT: Camera Repair (Reparación de Cámaras)
==========================================
Procedimiento estándar para diagnosticar y reparar problemas de cámaras.

Triggers:
- check_id: camera_*, vision_camera, cam_*
- keywords: camera, cámara, webcam, opencv, video capture

Severidad: MEDIUM (puede afectar visión pero no es crítico para el sistema)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="camera_repair",
        name="Reparación de Cámaras",
        description="""
Procedimiento para diagnosticar y reparar problemas con cámaras USB/integradas.
Cubre detección de índices, cambio de backend OpenCV, reinicio de drivers y
verificación de frames.
        """.strip(),
        category=POTCategory.REPAIR,
        severity=POTSeverity.MEDIUM,
        version="2.0.0",
        author="ATLAS QA Senior",
        
        # Triggers
        trigger_check_ids=["camera_health", "vision_camera", "cam_check", "camera_*"],
        trigger_keywords=["camera", "cámara", "webcam", "opencv", "video", "capture", "frame"],
        
        # Requisitos
        prerequisites=[
            "Robot backend debe estar corriendo (port 8002)",
            "Push dashboard debe estar accesible (port 8791)",
            "Al menos una cámara USB conectada físicamente",
        ],
        required_services=["robot", "push"],
        required_permissions=["camera_control", "service_restart"],
        
        # Objetivos
        objectives=[
            "Identificar cámaras disponibles en el sistema",
            "Diagnosticar cuál cámara está fallando",
            "Aplicar corrección (backend, índice, reinicio)",
            "Verificar que al menos una cámara capture frames",
        ],
        success_criteria="Al menos una cámara devuelve frame_ok=true y snapshot válido",
        estimated_duration_minutes=5,
        
        # Tutorial
        tutorial_overview="""
## Guía de Reparación de Cámaras

### Contexto
Las cámaras en Windows pueden fallar por:
1. **Conflicto de backend**: MSMF vs DSHOW (preferir DSHOW)
2. **Índice incorrecto**: La cámara cambió de índice tras reconexión
3. **Driver bloqueado**: Otra aplicación tiene la cámara abierta
4. **Timeout**: Cámara lenta al inicializar

### Estrategia
1. Primero diagnosticamos: escaneamos índices 0-3
2. Probamos cada cámara en subprocess aislado (evita crash del backend)
3. Seleccionamos la mejor cámara funcional
4. Aplicamos configuración y verificamos
        """.strip(),
        
        best_practices=[
            "Siempre usar subprocess aislado para probar cámaras",
            "Preferir CAP_DSHOW sobre CAP_MSMF en Windows",
            "Liberar cámara (release) antes de cambiar de índice",
            "Dar tiempo de inicialización (2-3 segundos) a cámaras lentas",
            "Cerrar aplicaciones que puedan estar usando la cámara",
        ],
        
        warnings=[
            "NO abrir múltiples cámaras simultáneamente",
            "Si el robot backend crashea, el subprocess lo protegerá",
            "Algunos drivers escriben logs a stdout (manejar JSON parsing)",
        ],
        
        related_pots=["services_repair", "vision_calibration", "diagnostic_full"],
        tags=["camera", "vision", "opencv", "hardware", "repair"],
        has_rollback=True,
        
        # === PASOS DEL PROCEDIMIENTO ===
        steps=[
            POTStep(
                id="check_services",
                name="Verificar servicios activos",
                description="Confirmar que Robot y Push responden antes de iniciar",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8002/api/health",
                timeout_seconds=10,
                tutorial_notes="""
Antes de tocar cámaras, verificamos que los servicios base estén OK.
Si el Robot no responde, no tiene sentido continuar con cámaras.
                """,
                common_errors=["Connection refused = Robot no está corriendo"],
                troubleshooting="Ejecutar: powershell scripts/restart_service_clean.ps1 -Service robot",
            ),
            
            POTStep(
                id="close_active_streams",
                name="Cerrar streams activos",
                description="Liberar recursos de cámara antes de diagnosticar",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8002/api/camera/disconnect",
                http_body={"index": -1},  # -1 = todas
                timeout_seconds=15,
                continue_on_failure=True,
                tutorial_notes="""
Es crucial liberar cualquier cámara abierta antes de escanear.
Evita conflictos de "device busy" y permite probar cada índice limpiamente.
                """,
            ),
            
            POTStep(
                id="wait_device_release",
                name="Esperar liberación de dispositivos",
                description="Dar tiempo al sistema para liberar handles",
                step_type=StepType.WAIT,
                wait_seconds=2,
                tutorial_notes="Windows necesita ~1-2 segundos para liberar handles de cámara.",
            ),
            
            POTStep(
                id="scan_cameras",
                name="Escanear índices de cámara",
                description="Detectar qué índices tienen cámara conectada",
                step_type=StepType.SCRIPT,
                script_path="scripts/atlas_camera_autorepair.py",
                timeout_seconds=120,
                capture_output=True,
                tutorial_notes="""
El script de auto-reparación:
1. Prueba índices 0, 1, 2 en subprocess aislado
2. Para cada uno intenta abrir con CAP_DSHOW
3. Verifica si puede leer un frame
4. Selecciona el mejor índice funcional
5. Actualiza active_camera.json
                """,
                common_errors=[
                    "probe_output_parse_error = Driver escribió basura a stdout",
                    "open_failed = Cámara no existe o está ocupada",
                    "timeout = Cámara muy lenta o driver colgado",
                ],
            ),
            
            POTStep(
                id="verify_selected_camera",
                name="Verificar cámara seleccionada",
                description="Confirmar que la cámara elegida funciona",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/cuerpo/api/camera/status",
                timeout_seconds=10,
                check_expression="response.get('active_index') is not None",
                tutorial_notes="""
Después del scan, verificamos que el sistema reporta una cámara activa.
El campo active_index debe tener un número (0, 1 o 2 típicamente).
                """,
            ),
            
            POTStep(
                id="capture_test_snapshot",
                name="Capturar snapshot de prueba",
                description="Tomar una imagen para verificar que hay video real",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/cuerpo/vision/snapshot?source=camera&jpeg_quality=80",
                timeout_seconds=20,
                check_expression="response_status == 200 and len(response_body) > 5000",
                tutorial_notes="""
El snapshot debe:
- Devolver HTTP 200
- Tener más de 5KB (una imagen real, no un placeholder)
Si falla, la cámara conecta pero no produce frames válidos.
                """,
                common_errors=[
                    "404 = Ruta incorrecta o cámara no configurada",
                    "500 = Error interno al capturar",
                    "Imagen pequeña (<5KB) = Frame corrupto o negro",
                ],
            ),
            
            POTStep(
                id="log_success",
                name="Registrar éxito en bitácora",
                description="Notificar al ANS que la cámara fue reparada",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[REPARACIÓN] Cámara reparada exitosamente por POT camera_repair",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
                tutorial_notes="Siempre registrar en bitácora para trazabilidad.",
            ),
            
            POTStep(
                id="notify_success",
                name="Notificar éxito",
                description="Enviar notificación de reparación completada",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="✅ POT camera_repair completado exitosamente. Cámara operativa.",
                continue_on_failure=True,
            ),
        ],
        
        # === PASOS DE ROLLBACK ===
        rollback_steps=[
            POTStep(
                id="rollback_reset_index",
                name="Restaurar índice por defecto",
                description="Volver a índice 0 si algo salió mal",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8002/api/camera/connect-index",
                http_body={"index": 0, "resolution": [640, 480]},
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_restart_robot",
                name="Reiniciar Robot backend",
                description="Si el driver quedó corrupto, reiniciar el servicio",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
            ),
            
            POTStep(
                id="rollback_notify",
                name="Notificar rollback",
                description="Informar que se ejecutó rollback",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="⚠️ POT camera_repair ejecutó ROLLBACK. Revisar manualmente.",
            ),
        ],
    )
