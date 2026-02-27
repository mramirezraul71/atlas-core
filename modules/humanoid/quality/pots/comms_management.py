"""
POT: Communications Management (Gestión de Comunicaciones)
===========================================================
Procedimiento completo para gestionar el sistema de comunicaciones de ATLAS:
WhatsApp (WAHA), Telegram, Audio (TTS).

Triggers:
- check_id: comms_*, whatsapp_*, telegram_*, audio_*, waha_*
- keywords: whatsapp, telegram, waha, audio, comunicacion, mensaje, notificacion

Severidad: HIGH (comunicaciones son críticas para operación autónoma)

Este POT enseña a ATLAS a:
1. Verificar estado de todos los canales de comunicación
2. Diagnosticar problemas de conexión con WAHA
3. Reiniciar sesiones de WhatsApp
4. Enviar mensajes de prueba
5. Gestionar la bitácora de comunicaciones
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="comms_management",
        name="Gestión de Comunicaciones",
        description="""
Procedimiento integral para gestionar el sistema de comunicaciones multi-canal de ATLAS.

CANALES DISPONIBLES:
- WhatsApp (WAHA): Mensajería instantánea vía WhatsApp HTTP API (self-hosted)
- Telegram: Bot de Telegram para notificaciones y comandos
- Audio (TTS): Text-to-Speech para notificaciones audibles locales

ARQUITECTURA:
```
                    ┌─────────────────────┐
                    │     CommsHub        │
                    │ (Orquestador Central)│
                    └──────────┬──────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
┌────────────────┐  ┌────────────────────┐  ┌────────────────┐
│ WhatsApp Bridge│  │  Telegram Bridge   │  │  Audio Bridge  │
│  (WAHA Docker) │  │   (Bot API)        │  │  (pyttsx3)     │
│  Puerto: 3010  │  │                    │  │                │
└────────────────┘  └────────────────────┘  └────────────────┘
```

NIVELES DE PRIORIDAD:
- INFO/LOW: Solo log
- MEDIUM: Log + Audio
- HIGH: Log + Audio + Telegram  
- CRITICAL: Log + Audio + Telegram + WhatsApp
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS Communications Team",
        
        trigger_check_ids=["comms_health", "whatsapp_health", "telegram_health", "waha_*", "audio_*"],
        trigger_keywords=[
            "whatsapp", "waha", "telegram", "audio", "comunicacion", "mensaje", 
            "notificacion", "bot", "tts", "speech", "qr", "sesion"
        ],
        
        prerequisites=[
            "Docker instalado y corriendo (para WAHA)",
            "Contenedor WAHA desplegado en puerto 3010",
            "Token de Telegram configurado en credenciales.txt (opcional)",
            "pyttsx3 instalado para audio (pip install pyttsx3)",
        ],
        required_services=["docker"],
        required_permissions=["docker_exec", "http_requests"],
        
        objectives=[
            "Verificar estado de todos los canales de comunicación",
            "Diagnosticar y reparar conexión de WAHA/WhatsApp",
            "Enviar mensajes de prueba para verificar funcionalidad",
            "Documentar estado en la bitácora central",
        ],
        success_criteria="Todos los canales configurados responden correctamente a mensajes de prueba",
        estimated_duration_minutes=5,
        
        tutorial_overview="""
## Sistema de Comunicaciones ATLAS

### Componentes Principales

#### 1. WAHA (WhatsApp HTTP API)
- **Qué es**: Servidor Docker que permite enviar/recibir WhatsApp sin cuenta Business
- **Puerto**: 3010 (configurable via WAHA_API_URL)
- **Dashboard**: http://localhost:3010 (acceso directo para gestión)
- **API Key**: atlas123 (configurable via WAHA_API_KEY)

#### 2. Telegram Bridge
- **Qué es**: Conector al Bot API de Telegram
- **Requisitos**: TELEGRAM_BOT_TOKEN y TELEGRAM_CHAT_ID en credenciales.txt
- **Uso**: Notificaciones automáticas y comandos de aprobación

#### 3. Audio Bridge (TTS)
- **Qué es**: Text-to-Speech local usando pyttsx3
- **Uso**: Notificaciones audibles en el equipo local

### Credenciales Necesarias (credenciales.txt)

```
# WhatsApp (WAHA)
WAHA_API_URL=http://localhost:3010
WAHA_API_KEY=atlas123
WAHA_TO_NUMBER=+1234567890  # Tu número de WhatsApp

# Telegram
TELEGRAM_BOT_TOKEN=123456789:ABC...
TELEGRAM_CHAT_ID=-100123456789

# Audio
AUDIO_ENABLED=1
```

### Endpoints de API

| Endpoint | Método | Descripción |
|----------|--------|-------------|
| /ans/comms/status | GET | Estado de todos los canales |
| /ans/comms/waha/qr | GET | Obtener QR para vincular WhatsApp |
| /ans/comms/waha/restart | POST | Reiniciar sesión de WAHA |
| /ans/comms/test | POST | Enviar mensaje de prueba |
| /api/comms/whatsapp/send | POST | Enviar mensaje por WhatsApp |

### Flujo de Vinculación de WhatsApp

1. Iniciar contenedor WAHA: `docker start waha`
2. Verificar estado: GET http://localhost:3010/api/sessions/default
3. Si no está autenticado, obtener QR: GET /api/default/auth/qr
4. Escanear QR con WhatsApp en el teléfono
5. Verificar autenticación: status debe ser "WORKING"

### Troubleshooting

| Problema | Causa | Solución |
|----------|-------|----------|
| WhatsApp no conecta | QR expirado | Regenerar QR y escanear rápido |
| WAHA no responde | Docker caído | `docker start waha` |
| Telegram no envía | Token inválido | Verificar TELEGRAM_BOT_TOKEN |
| Audio no funciona | pyttsx3 no instalado | `pip install pyttsx3` |
        """.strip(),
        
        best_practices=[
            "Verificar estado de WAHA antes de enviar mensajes importantes",
            "Usar el Dashboard de WAHA para diagnóstico visual rápido",
            "Mantener WhatsApp vinculado permanentemente para evitar re-escaneos",
            "Probar canales periódicamente con mensajes de diagnóstico",
            "Registrar todos los envíos importantes en la bitácora",
            "Usar niveles de prioridad apropiados (no todo es CRITICAL)",
        ],
        
        warnings=[
            "WAHA requiere Docker corriendo - verificar antes de usar",
            "El QR de WhatsApp expira en ~60 segundos - escanear rápido",
            "No enviar mensajes en bucle - respetar rate limits",
            "WhatsApp puede desconectarse si el teléfono pierde internet",
            "Telegram tiene límite de mensajes por minuto",
        ],
        
        related_pots=["services_repair", "notification_broadcast", "diagnostic_full"],
        tags=["comms", "whatsapp", "telegram", "waha", "audio", "notifications", "messaging"],
        has_rollback=False,
        
        steps=[
            # ============ VERIFICACIÓN INICIAL ============
            POTStep(
                id="check_docker",
                name="Verificar Docker",
                description="Comprobar que Docker está corriendo (necesario para WAHA)",
                step_type=StepType.SHELL,
                shell_command="docker info --format '{{.ServerVersion}}'",
                expected_output="Docker version",
                on_failure="skip_waha",
                timeout_seconds=10,
            ),
            
            POTStep(
                id="check_waha_container",
                name="Verificar Contenedor WAHA",
                description="Comprobar que el contenedor WAHA está corriendo",
                step_type=StepType.SHELL,
                shell_command="docker ps --filter name=waha --format '{{.Status}}'",
                expected_output="Up",
                on_failure="start_waha_container",
                timeout_seconds=10,
            ),
            
            POTStep(
                id="start_waha_container",
                name="Iniciar Contenedor WAHA",
                description="Iniciar el contenedor WAHA si no está corriendo",
                step_type=StepType.SHELL,
                shell_command="docker start waha",
                on_failure="fail",
                timeout_seconds=30,
            ),
            
            # ============ ESTADO DE CANALES ============
            POTStep(
                id="check_comms_status",
                name="Obtener Estado de Comunicaciones",
                description="Consultar el estado de todos los canales via API",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/status",
                http_method="GET",
                expected_status=200,
                on_failure="log_comms_error",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="check_whatsapp_session",
                name="Verificar Sesión WhatsApp",
                description="Comprobar que WAHA está autenticado con WhatsApp",
                step_type=StepType.HTTP,
                http_url="http://localhost:3010/api/sessions/default",
                http_method="GET",
                http_headers={"X-Api-Key": "atlas123"},
                expected_status=200,
                on_failure="restart_waha_session",
                timeout_seconds=10,
            ),
            
            # ============ REPARACIÓN WAHA ============
            POTStep(
                id="restart_waha_session",
                name="Reiniciar Sesión WAHA",
                description="Detener y reiniciar la sesión de WAHA para forzar nuevo QR",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/waha/restart",
                http_method="POST",
                expected_status=200,
                on_failure="manual_waha_intervention",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="get_whatsapp_qr",
                name="Obtener QR de WhatsApp",
                description="Obtener el código QR para vincular WhatsApp (si es necesario)",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/waha/qr",
                http_method="GET",
                expected_status=200,
                on_failure="log_qr_error",
                timeout_seconds=10,
            ),
            
            # ============ PRUEBAS DE ENVÍO ============
            POTStep(
                id="test_telegram",
                name="Probar Envío Telegram",
                description="Enviar mensaje de prueba por Telegram",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/test?channel=telegram&message=POT%20Comms%20Test%20-%20Telegram%20OK",
                http_method="POST",
                expected_status=200,
                on_failure="log_telegram_error",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="test_whatsapp",
                name="Probar Envío WhatsApp",
                description="Enviar mensaje de prueba por WhatsApp",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/test?channel=whatsapp&message=POT%20Comms%20Test%20-%20WhatsApp%20OK",
                http_method="POST",
                expected_status=200,
                on_failure="log_whatsapp_error",
                timeout_seconds=15,
            ),
            
            POTStep(
                id="test_audio",
                name="Probar Audio TTS",
                description="Reproducir mensaje de prueba por audio",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/comms/test?channel=audio&message=Sistema%20de%20comunicaciones%20verificado",
                http_method="POST",
                expected_status=200,
                on_failure="log_audio_error",
                timeout_seconds=10,
            ),
            
            # ============ REGISTRO EN BITÁCORA ============
            POTStep(
                id="log_success",
                name="Registrar Éxito en Bitácora",
                description="Documentar la verificación exitosa en la bitácora central",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[COMMS] Sistema de comunicaciones verificado - Todos los canales operativos", "ok": True, "source": "comms"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            # ============ PASOS DE ERROR/FALLBACK ============
            POTStep(
                id="log_comms_error",
                name="Registrar Error de Comunicaciones",
                description="Documentar error en la bitácora",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[COMMS] Error verificando sistema de comunicaciones", "ok": False, "source": "comms"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="log_telegram_error",
                name="Registrar Error Telegram",
                description="Documentar fallo de Telegram",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[TELEGRAM] Error enviando mensaje - verificar token y chat_id", "ok": False, "source": "telegram"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="log_whatsapp_error",
                name="Registrar Error WhatsApp",
                description="Documentar fallo de WhatsApp",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[WHATSAPP] Error enviando mensaje - verificar sesión WAHA y QR", "ok": False, "source": "whatsapp"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="log_audio_error",
                name="Registrar Error Audio",
                description="Documentar fallo de Audio",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[AUDIO] Error reproduciendo audio - verificar pyttsx3", "ok": False, "source": "audio"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="log_qr_error",
                name="Registrar Error QR",
                description="Documentar que no se pudo obtener QR",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[WAHA] No se pudo obtener QR - WhatsApp puede estar ya vinculado", "ok": True, "source": "whatsapp"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="manual_waha_intervention",
                name="Intervención Manual WAHA",
                description="Requiere intervención manual para vincular WhatsApp",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[WAHA] Requiere intervención manual - abrir http://localhost:3010 y escanear QR", "ok": False, "source": "whatsapp"},
                expected_status=200,
                timeout_seconds=5,
            ),
            
            POTStep(
                id="skip_waha",
                name="Omitir WAHA (Docker no disponible)",
                description="Docker no está corriendo, omitir verificación de WAHA",
                step_type=StepType.HTTP,
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_method="POST",
                http_body={"message": "[WAHA] Docker no disponible - WhatsApp deshabilitado temporalmente", "ok": False, "source": "whatsapp"},
                expected_status=200,
                timeout_seconds=5,
            ),
        ],
        
        # Documentación adicional para ATLAS
        additional_notes="""
## Uso Programático

### Enviar Mensaje por WhatsApp (Python)
```python
from modules.humanoid.comms.whatsapp_bridge import send_text

# Enviar mensaje simple
result = send_text("Hola desde ATLAS")
print(f"Enviado: {result.get('ok')}")
```

### Enviar por Telegram (Python)
```python
from modules.humanoid.comms.telegram_bridge import send

# Enviar mensaje
ok = send("Notificación desde ATLAS")
```

### Enviar por Audio (Python)
```python
from modules.humanoid.comms.ops_bus import emit

# Hablar mensaje
emit("Mensaje de prueba", subsystem="audio", severity="info")
```

### Enviar a Múltiples Canales (CommsHub)
```python
from modules.humanoid.comms.hub import hub_emit, MessageLevel

# Enviar a todos los canales según prioridad
hub_emit(
    message="Mensaje importante",
    level=MessageLevel.HIGH,  # Telegram + Audio + Log
    source="mi_modulo"
)
```

## Configuración de Variables de Entorno

```bash
# WAHA (WhatsApp)
WAHA_API_URL=http://localhost:3010
WAHA_API_KEY=atlas123
WAHA_TO_NUMBER=+19192078141
WHATSAPP_ENABLED=1

# Telegram
TELEGRAM_BOT_TOKEN=tu_token_aqui
TELEGRAM_CHAT_ID=tu_chat_id_aqui
OPS_TELEGRAM_ENABLED=1

# Audio
AUDIO_ENABLED=1
OPS_AUDIO_ENABLED=1
```

## Dashboard WAHA

Acceder a http://localhost:3010 para:
- Ver estado de la sesión
- Escanear QR para vincular
- Ver logs de mensajes
- Gestionar webhook

Credenciales por defecto:
- Usuario: admin
- Password: Ver logs de docker con `docker logs waha`
        """.strip(),
    )


# Función para verificación rápida
def quick_check() -> dict:
    """Verificación rápida del estado de comunicaciones."""
    import urllib.request
    import json
    
    result = {"ok": True, "channels": {}}
    
    # WAHA
    try:
        req = urllib.request.Request(
            "http://localhost:3010/api/sessions/default",
            headers={"X-Api-Key": "atlas123"}
        )
        with urllib.request.urlopen(req, timeout=5) as r:
            data = json.loads(r.read().decode())
            result["channels"]["whatsapp"] = {
                "ok": data.get("status") == "WORKING",
                "status": data.get("status"),
            }
    except Exception as e:
        result["channels"]["whatsapp"] = {"ok": False, "error": str(e)}
        result["ok"] = False
    
    # Dashboard
    try:
        req = urllib.request.Request("http://127.0.0.1:8791/health")
        with urllib.request.urlopen(req, timeout=5) as r:
            result["channels"]["dashboard"] = {"ok": True}
    except Exception as e:
        result["channels"]["dashboard"] = {"ok": False, "error": str(e)}
        result["ok"] = False
    
    return result
