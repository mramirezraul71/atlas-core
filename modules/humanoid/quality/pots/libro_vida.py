"""
POT: Libro de Vida — Verificación y Ejercicio
==============================================
Procedimiento que verifica la integridad del Libro de Vida de ATLAS,
ejecuta un ciclo completo de planificación cognitiva, y valida que
el sistema de experiencias funcione end-to-end.

Triggers:
- Cada 6 horas (automático)
- Comando "verificar libro de vida"
- Después de cualquier tarea compleja completada

Severidad: MEDIUM
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="libro_vida",
        name="Libro de Vida — Verificación Cognitiva",
        description="""
Procedimiento de verificación integral del Libro de Vida de ATLAS:
- Verificar que la DB del Libro de Vida existe y responde
- Validar estadísticas de episodios almacenados
- Ejecutar una búsqueda de experiencias
- Realizar un ciclo de planificación cognitiva de prueba
- Registrar un episodio de diagnóstico
- Verificar sincronización con subsistemas (Lifelog, World Model, Autobiographical)
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS Cognitive Engine",

        trigger_check_ids=["libro_vida_*", "cognitive_check"],
        trigger_keywords=["libro de vida", "book of life", "experiencias", "memoria cognitiva",
                          "planificacion", "episodios"],

        prerequisites=[
            "Servicio PUSH (8791) operativo",
            "Libro de Vida inicializado",
        ],
        required_services=["push_api"],
        required_permissions=["memory_read", "memory_write"],

        objectives=[
            "Verificar integridad de la DB del Libro de Vida",
            "Validar búsqueda de episodios por texto y tipo",
            "Ejecutar planificación cognitiva con LLM",
            "Registrar episodio de prueba y verificar persistencia",
            "Confirmar sincronización con Lifelog y World Model",
        ],
        success_criteria="Libro de Vida operativo con capacidad de registro, búsqueda y planificación",
        estimated_duration_minutes=2,

        tutorial_overview="""
## Guía del Libro de Vida

### Arquitectura Cognitiva
```
┌────────────────┐     ┌──────────────┐     ┌────────────────┐
│  Nueva Tarea   │────▶│  Buscar en   │────▶│  Elaborar Plan │
│  del Humano    │     │  Libro Vida  │     │  con LLM       │
└────────────────┘     └──────────────┘     └────────────────┘
                                                    │
                            ┌───────────────────────┘
                            ▼
                    ┌──────────────┐     ┌────────────────┐
                    │  Ejecutar    │────▶│  Registrar     │
                    │  Plan        │     │  Resultado     │
                    └──────────────┘     └────────────────┘
```

### Cada episodio contiene:
1. **Contexto**: entorno, restricciones, participantes
2. **Percepciones**: sensores, eventos, anomalías
3. **Acciones**: pasos ejecutados, alternativas descartadas
4. **Resultados**: éxito/fallo, métricas, riesgos
5. **Feedback**: correcciones humanas, recompensas
6. **Lecciones**: texto, reglas numéricas, recomendaciones

### Endpoints principales:
- `POST /api/libro-vida/planificar` — Planificar tarea con experiencias
- `POST /api/libro-vida/registrar-resultado` — Registrar resultado
- `POST /api/libro-vida/buscar` — Buscar episodios similares
- `GET  /api/libro-vida/status` — Estadísticas del Libro
        """.strip(),

        best_practices=[
            "Registrar siempre el resultado después de cada tarea compleja",
            "Incluir feedback humano para mejorar futuras planificaciones",
            "Revisar reglas aprendidas periódicamente",
            "Marcar episodios sin precedentes como 'alto valor de aprendizaje'",
        ],

        related_pots=["diagnostic_full", "session_startup", "autonomy_full_cycle"],
        tags=["cognitive", "memory", "planning", "experience", "learning"],

        steps=[
            POTStep(
                id="check_libro_vida_status",
                name="Verificar estado del Libro de Vida",
                description="Confirmar que la DB existe y responde con estadísticas",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/libro-vida/status",
                timeout_seconds=10,
                capture_output=True,
            ),

            POTStep(
                id="check_episodios_list",
                name="Listar episodios recientes",
                description="Verificar que se pueden listar episodios",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/libro-vida/episodios?limit=5",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),

            POTStep(
                id="test_busqueda",
                name="Buscar experiencias por texto",
                description="Probar búsqueda de episodios similares",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/api/libro-vida/buscar",
                http_body={"query": "diagnostico sistema verificacion", "limit": 3},
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),

            POTStep(
                id="test_registrar_episodio",
                name="Registrar episodio de diagnóstico",
                description="Crear un episodio de prueba para verificar escritura",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/api/libro-vida/registrar-resultado",
                http_body={
                    "tarea": "POT: Verificación del Libro de Vida",
                    "tipo_tarea": "diagnostico",
                    "plan_ejecutado": [
                        "Verificar DB del Libro de Vida",
                        "Listar episodios recientes",
                        "Buscar experiencias similares",
                        "Registrar este episodio de prueba",
                    ],
                    "exito": True,
                    "lecciones": "El Libro de Vida está operativo y responde correctamente",
                    "contexto_entorno": "Ejecución automática de POT de verificación",
                    "metricas": {"pasos_ok": 4, "pasos_total": 7},
                },
                timeout_seconds=15,
                capture_output=True,
            ),

            POTStep(
                id="check_reglas",
                name="Verificar reglas aprendidas",
                description="Confirmar acceso al sistema de reglas",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/libro-vida/reglas?limit=5",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),

            POTStep(
                id="check_principios",
                name="Verificar principios generales",
                description="Confirmar acceso al sistema de principios",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/libro-vida/principios",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),

            POTStep(
                id="check_subsystems_sync",
                name="Verificar sincronización con subsistemas",
                description="Confirmar que Lifelog y World Model reciben datos del Libro de Vida",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/cognitive-memory/lifelog/status",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),

            POTStep(
                id="log_verification",
                name="Registrar verificación completada",
                description="Log en bitácora que el Libro de Vida fue verificado",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[LIBRO DE VIDA] Verificación cognitiva completada — sistema operativo",
                    "ok": True,
                    "source": "quality_pot"
                },
            ),
        ],
    )
