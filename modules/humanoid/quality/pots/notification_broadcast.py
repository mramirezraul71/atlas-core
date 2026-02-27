"""
POT: Notification Broadcast (Difusión de Notificaciones)
========================================================
Procedimiento para enviar notificaciones a todos los canales.

Triggers:
- Eventos importantes del sistema
- Alertas críticas
- Completación de operaciones mayores

Severidad: LOW (solo envía información)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="notification_broadcast",
        name="Difusión de Notificaciones",
        description="""
Procedimiento para enviar notificaciones a todos los canales de comunicación:
- Telegram (Owner)
- OPS Bus (Sistema interno)
- Bitácora ANS (Registro permanente)
- Dashboard (Actualización visual)
        """.strip(),
        category=POTCategory.DIAGNOSTIC,  # No modifica sistema
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["notify_*", "broadcast_*", "alert_*"],
        trigger_keywords=["notify", "notification", "broadcast", "alert", "message"],
        
        prerequisites=[
            "Canales de comunicación configurados",
            "Mensaje a enviar definido",
        ],
        required_services=["push"],
        required_permissions=["notify"],
        
        objectives=[
            "Enviar mensaje a Telegram",
            "Registrar en OPS Bus",
            "Guardar en Bitácora ANS",
            "Actualizar estado en Dashboard",
        ],
        success_criteria="Mensaje entregado a al menos un canal",
        estimated_duration_minutes=1,
        
        tutorial_overview="""
## Guía de Difusión de Notificaciones

### Canales Disponibles
```
┌─────────────────────────────────────────────────┐
│              ATLAS Notification Hub              │
├─────────────────────────────────────────────────┤
│                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ Telegram │  │ OPS Bus  │  │ Bitácora │       │
│  │          │  │          │  │   ANS    │       │
│  │ • Owner  │  │ • Interno│  │ • Log    │       │
│  │ • Admins │  │ • Eventos│  │ • Audit  │       │
│  └──────────┘  └──────────┘  └──────────┘       │
│                                                  │
│  ┌──────────┐  ┌──────────┐                     │
│  │Dashboard │  │ Console  │                     │
│  │          │  │          │                     │
│  │ • Visual │  │ • Debug  │                     │
│  │ • Real-  │  │ • Dev    │                     │
│  │   time   │  │          │                     │
│  └──────────┘  └──────────┘                     │
│                                                  │
└─────────────────────────────────────────────────┘
```

### Niveles de Mensaje
- **info**: Informativo (azul)
- **low**: Baja prioridad (verde)
- **med**: Media prioridad (amarillo)
- **high**: Alta prioridad (naranja)
- **critical**: Crítico (rojo)
        """.strip(),
        
        best_practices=[
            "Usar nivel apropiado para el mensaje",
            "Incluir contexto suficiente",
            "No saturar canales con mensajes triviales",
            "Usar emojis para contexto visual rápido",
        ],
        
        related_pots=["incident_response", "deployment_full"],
        tags=["notification", "telegram", "ops", "broadcast", "communication"],
        
        steps=[
            POTStep(
                id="prepare_message",
                name="Preparar mensaje",
                description="Formatear mensaje para envío",
                step_type=StepType.LOG,
                tutorial_notes="""
El mensaje viene del contexto:
context['notification_message'] = "Mi mensaje"
context['notification_level'] = "info"
                """,
            ),
            
            POTStep(
                id="send_telegram",
                name="Enviar a Telegram",
                description="Notificar al owner vía Telegram",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="📢 Notificación del sistema ATLAS",
                continue_on_failure=True,
            ),
            
            POTStep(
                id="send_ops",
                name="Enviar a OPS Bus",
                description="Registrar en canal OPS interno",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="Sistema: Notificación broadcast",
                continue_on_failure=True,
            ),
            
            POTStep(
                id="log_bitacora",
                name="Registrar en Bitácora",
                description="Guardar en log permanente ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[BROADCAST] Notificación enviada a todos los canales",
                    "ok": True,
                    "source": "quality_pot"
                },
            ),
            
            POTStep(
                id="refresh_dashboard",
                name="Refrescar Dashboard",
                description="Actualizar estado en UI",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/status",
                timeout_seconds=10,
                continue_on_failure=True,
            ),
        ],
    )
