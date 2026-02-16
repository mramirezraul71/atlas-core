"""
POT: Visita de Especialista (Sistema de Tutorías)
==================================================
Procedimiento para gestionar visitas de especialistas a ATLAS.

Triggers:
- Al llegar un nuevo especialista
- Comando "nueva visita" o "iniciar tutoría"
- Acceso al tab de Tutorías en dashboard

Severidad: MEDIUM

IMPORTANTE: Todo especialista que instruya a ATLAS debe registrarse
y dejar constancia de sus recomendaciones mediante este sistema.
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="specialist_visit",
        name="Visita de Especialista - Sistema de Tutorías",
        description="""
Sistema de registro, seguimiento y documentación de visitas de especialistas.

PROPÓSITO:
Cada persona que instruya, configure o modifique ATLAS debe:
1. Registrarse como especialista (si es primera vez)
2. Iniciar una visita formal
3. Documentar sus instrucciones y evaluaciones
4. Firmar digitalmente su informe
5. Dejar recomendaciones de mejora

TIPOS DE VISITA:
- 📚 Tutoría: Enseñanza y configuración
- 🔍 Revisión: Inspección de código/funcionamiento
- 📊 Auditoría: Evaluación formal de calidad
- 🎓 Capacitación: Entrenamiento del sistema
- 🔧 Mantenimiento: Correcciones y ajustes
- 🚨 Emergencia: Reparación urgente
- 📌 Seguimiento: Verificación de mejoras previas

ALMACENAMIENTO:
- Base de datos: data/quality/tutorias.db
- Reportes: data/reports/tutorias/
        """.strip(),
        category=POTCategory.QUALITY,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS Quality System",
        
        trigger_check_ids=["specialist_*", "visit_*", "tutoria_*"],
        trigger_keywords=[
            "visita", "especialista", "tutoria", "auditoria",
            "visit", "specialist", "training", "audit",
            "nuevo especialista", "nueva visita", "registrar visita"
        ],
        
        prerequisites=[
            "Sistema ATLAS operativo",
            "API en http://127.0.0.1:8791",
            "Dashboard accesible",
        ],
        required_services=["atlas_api"],
        required_permissions=["tutorias_write", "tutorias_read"],
        
        objectives=[
            "Registrar especialista si es nuevo",
            "Iniciar visita con tipo y motivo",
            "Documentar trabajo realizado",
            "Evaluar aspectos del sistema",
            "Crear recomendaciones de mejora",
            "Firmar informe digitalmente",
            "Generar reportes de seguimiento",
        ],
        success_criteria="Visita completada con informe firmado y recomendaciones registradas",
        estimated_duration_minutes=30,
        
        tutorial_overview="""
## 📋 Sistema de Tutorías y Visitas de Especialistas

### ¿Qué es?
Sistema formal para documentar TODAS las instrucciones y modificaciones
que cualquier especialista realice sobre ATLAS.

### ¿Por qué es obligatorio?
1. **Trazabilidad**: Saber quién hizo qué y cuándo
2. **Calidad**: Evaluar el estado del sistema periódicamente
3. **Mejora Continua**: Seguimiento de recomendaciones
4. **Responsabilidad**: Firma digital en cada informe

### Flujo de una Visita

```
┌─────────────────┐
│  ESPECIALISTA   │
│    LLEGA        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐     ¿Primera    ┌─────────────────┐
│  ¿Ya está       │───── vez? ─────▶│   REGISTRARSE   │
│  registrado?    │      SÍ         │  (nombre, rol,  │
└────────┬────────┘                 │   especialidad) │
         │ NO                       └────────┬────────┘
         │◀─────────────────────────────────┘
         ▼
┌─────────────────┐
│ INICIAR VISITA  │
│ (tipo, motivo,  │
│  módulos)       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ REALIZAR TRABAJO│
│ (tutoría, audit,│
│  revisión, etc) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ CREAR INFORME   │
│ - Evaluaciones  │
│ - Recomendaciones│
│ - Próximos pasos│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ FIRMAR Y CERRAR │
│ (firma digital) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ GENERAR REPORTE │
│ (MD/HTML/JSON)  │
└─────────────────┘
```

### Acceso

**Dashboard**: Tab "📋 Tutorías"
**API Base**: http://127.0.0.1:8791/tutorias/

### Endpoints Principales

| Endpoint | Método | Descripción |
|----------|--------|-------------|
| /tutorias/especialistas | POST | Registrar nuevo especialista |
| /tutorias/especialistas | GET | Listar especialistas |
| /tutorias/visitas | POST | Iniciar visita |
| /tutorias/visitas/{id}/finalizar | POST | Finalizar con informe |
| /tutorias/recomendaciones | GET | Ver recomendaciones |
| /tutorias/seguimientos | GET | Ver seguimientos activos |
| /tutorias/estadisticas | GET | Estadísticas del sistema |

### Estructura de Datos

**Especialista**:
- nombre, rol, especialidad, email
- firma_digital (generada automáticamente)
- visitas_realizadas

**Visita**:
- tipo, motivo, módulos_revisados
- fecha_inicio, fecha_fin, duración
- informe (con firma)

**Informe**:
- evaluaciones (aspecto, nivel 1-5, comentario)
- recomendaciones (prioridad, módulo, pasos)
- firma_especialista, hash_verificación

### Comandos de Prueba

```bash
# Ver estadísticas
curl http://127.0.0.1:8791/tutorias/estadisticas

# Listar especialistas
curl http://127.0.0.1:8791/tutorias/especialistas

# Registrar especialista
curl -X POST http://127.0.0.1:8791/tutorias/especialistas \\
  -H "Content-Type: application/json" \\
  -d '{"nombre":"Dr. Juan","rol":"Arquitecto","especialidad":"Vision"}'

# Ver recomendaciones pendientes
curl http://127.0.0.1:8791/tutorias/recomendaciones?estado=PENDIENTE
```
        """.strip(),
        
        best_practices=[
            "SIEMPRE registrarse antes de hacer cambios",
            "Documentar TODO lo que se modifica",
            "Evaluar honestamente (no inflar puntuaciones)",
            "Crear recomendaciones específicas y accionables",
            "Firmar el informe antes de retirarse",
            "Revisar recomendaciones de visitas anteriores",
        ],
        
        related_pots=[
            "diagnostic_full",
            "maintenance_daily",
            "incident_response",
            "session_startup",
        ],
        tags=["tutoria", "especialista", "visita", "calidad", "informe", "firma", "evaluacion"],
        
        steps=[
            # Paso 1: Verificar sistema
            POTStep(
                id="check_system",
                name="Verificar sistema de tutorías",
                description="Confirmar que el sistema está disponible",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/tutorias/estadisticas",
                timeout_seconds=10,
                capture_output=True,
                expected_http_status=200,
            ),
            
            # Paso 2: Mostrar especialistas registrados
            POTStep(
                id="list_specialists",
                name="Listar especialistas registrados",
                description="Ver quiénes ya están en el sistema",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/tutorias/especialistas",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            # Paso 3: Instrucción para registro
            POTStep(
                id="register_instruction",
                name="Instrucción de registro",
                description="Guiar al especialista para registrarse si es nuevo",
                step_type=StepType.MANUAL,
                manual_instructions="""
## Registro de Especialista

Si es la primera vez que visita ATLAS, debe registrarse:

1. Ir al Dashboard → Tab "📋 Tutorías"
2. Completar el formulario "Registrar Nuevo Especialista":
   - Nombre completo
   - Rol (Arquitecto, Developer, QA, DevOps, etc.)
   - Especialidad (Vision, NLP, Robotics, etc.)
   - Email (opcional)
3. Click en "➕ Registrar Especialista"

Se generará automáticamente una FIRMA DIGITAL única.
                """,
                continue_on_failure=True,
            ),
            
            # Paso 4: Instrucción para iniciar visita
            POTStep(
                id="start_visit_instruction",
                name="Instrucción para iniciar visita",
                description="Guiar para crear una nueva visita",
                step_type=StepType.MANUAL,
                manual_instructions="""
## Iniciar Nueva Visita

1. Seleccionar su nombre en "Especialista"
2. Elegir tipo de visita:
   - 📚 Tutoría (enseñanza/configuración)
   - 🔍 Revisión (inspección)
   - 📊 Auditoría (evaluación formal)
   - 🎓 Capacitación (entrenamiento)
   - 🔧 Mantenimiento (correcciones)
   - 🚨 Emergencia (urgente)
   - 📌 Seguimiento (verificación)
3. Escribir el motivo de la visita
4. Marcar los módulos que revisará
5. Click en "🚀 Iniciar Visita"

Se iniciará un cronómetro de duración.
                """,
                continue_on_failure=True,
            ),
            
            # Paso 5: Durante la visita
            POTStep(
                id="during_visit",
                name="Durante la visita",
                description="Acciones durante el trabajo",
                step_type=StepType.MANUAL,
                manual_instructions="""
## Durante la Visita

Mientras trabaja con ATLAS:

1. **Documente** lo que hace en el campo "Notas"
2. **Observe** comportamientos para evaluar
3. **Identifique** áreas de mejora
4. **Anote** recomendaciones específicas

El sistema registra automáticamente:
- Tiempo transcurrido
- Módulos afectados
- Acciones en bitácora
                """,
                continue_on_failure=True,
            ),
            
            # Paso 6: Crear informe
            POTStep(
                id="create_report",
                name="Crear informe de visita",
                description="Documentar resultados de la visita",
                step_type=StepType.MANUAL,
                manual_instructions="""
## Crear Informe

Al finalizar el trabajo:

1. Click en "📝 Crear Informe"
2. Completar:
   - Título del informe
   - Resumen ejecutivo
   - Contenido detallado
   
3. Agregar EVALUACIONES:
   - Aspecto evaluado (ej: "Precisión Vision")
   - Nivel (Excelente/Bueno/Aceptable/Mejorable/Crítico)
   - Comentario

4. Agregar RECOMENDACIONES:
   - Título claro y específico
   - Prioridad (Crítica/Alta/Media/Baja/Opcional)
   - Descripción de qué hacer
   - Módulo afectado

5. Click en "✍️ Firmar y Enviar Informe"

⚠️ El informe quedará FIRMADO digitalmente con su firma única.
                """,
            ),
            
            # Paso 7: Verificar firma
            POTStep(
                id="verify_signature",
                name="Verificar firma digital",
                description="Confirmar que el informe quedó firmado",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/tutorias/informes?solo_firmados=true&limite=1",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            # Paso 8: Registrar en bitácora
            POTStep(
                id="log_visit",
                name="Registrar en bitácora central",
                description="Log de la visita completada",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/bitacora/log",
                http_body={
                    "message": "[TUTORIA] Visita de especialista completada y documentada",
                    "level": "success",
                    "source": "tutorias"
                },
                continue_on_failure=True,
            ),
            
            # Paso 9: Notificar
            POTStep(
                id="notify_completion",
                name="Notificar finalización",
                description="Avisar que la visita fue documentada",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="📋 Visita de especialista completada. Informe firmado y registrado.",
                continue_on_failure=True,
            ),
        ],
    )
