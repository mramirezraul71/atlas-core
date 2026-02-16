"""
POT: Human-Robot Interaction Operations
========================================
Procedimientos para interacción humano-robot.

Triggers:
- Inicio de interacción
- Detección de persona
- Comando de voz
- Gesto detectado

Severidad: HIGH (interacción con humanos)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="hri_interaction",
        name="Interacción Humano-Robot",
        description="""
Procedimiento para operaciones de HRI:
1. Detectar presencia humana
2. Establecer nivel de seguridad
3. Iniciar interfaz de comunicación
4. Procesar intenciones
5. Ejecutar acciones seguras
        """.strip(),
        category=POTCategory.COMMUNICATION,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["hri_*", "voice_*", "gesture_*", "safety_*"],
        trigger_keywords=["interacción", "voz", "gesto", "humano", "seguridad", "diálogo"],
        
        prerequisites=[
            "Cámaras y micrófonos activos",
            "Detección de personas funcionando",
            "Sistema de seguridad habilitado",
        ],
        required_services=["hri", "vision", "audio"],
        required_permissions=["communication", "movement"],
        
        objectives=[
            "Detectar y localizar humanos",
            "Mantener zonas de seguridad",
            "Procesar comandos de voz/gesto",
            "Mantener diálogo coherente",
            "Ejecutar acciones de forma segura",
        ],
        success_criteria="Interacción completada sin incidentes de seguridad",
        estimated_duration_minutes=5,
        
        tutorial_overview="""
## Guía de Interacción Humano-Robot ATLAS

### Arquitectura HRI
```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│    Voice     │  │   Gesture    │  │   Emotion    │
│  Interface   │  │   Recognizer │  │   Recognizer │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                  │
       ▼                 ▼                  ▼
┌─────────────────────────────────────────────────┐
│              INTENT PARSER                       │
│         (NLU + Gesture Commands)                 │
└───────────────────────┬─────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────┐
│              DIALOG MANAGER                      │
│         (Context + Response Gen)                 │
└───────────────────────┬─────────────────────────┘
                        │
           ┌────────────┴────────────┐
           ▼                         ▼
┌──────────────────┐      ┌──────────────────┐
│  SAFETY MONITOR  │      │   ACTION EXEC    │
│  (Zones, Limits) │      │   (Move, Fetch)  │
└──────────────────┘      └──────────────────┘
```

### Zonas de Seguridad (ISO 10218)
```
          2m         1m      0.5m    0.3m
    ┌──────────┬─────────┬────────┬───────┐
    │ NORMAL   │ CAUTION │WARNING │EMERG  │
    │ 0.5m/s   │ 0.3m/s  │ 0.1m/s │ STOP  │
    └──────────┴─────────┴────────┴───────┘
```

### Flujo de Comunicación
1. **Detectar**: Persona entra en zona de interacción
2. **Saludar**: Robot inicia contacto
3. **Escuchar**: Capturar voz/gestos
4. **Interpretar**: Extraer intención y entidades
5. **Responder**: Generar respuesta apropiada
6. **Actuar**: Ejecutar acción si es necesario
7. **Confirmar**: Verificar satisfacción

### Uso del Sistema HRI
```python
from modules.humanoid.hri import HRISystem, HRIConfig

# Crear sistema
config = HRIConfig(
    enable_voice=True,
    enable_gestures=True,
    enable_safety=True,
)
hri = HRISystem(config)
hri.start()

# Procesar texto
response = hri.process_text_input("Hola, tráeme un vaso")
print(response)

# Procesar visual (con personas detectadas)
results = hri.process_visual_input(
    rgb_frame,
    persons=[{"id": 1, "position": [1.0, 0, 0], "distance": 1.0}]
)
print(results["safety_level"])
```

### Gestos Reconocidos
- **WAVE**: Saludo
- **POINT**: Señalar dirección/objeto
- **THUMBS_UP**: Confirmación
- **STOP**: Parada
- **COME_HERE**: Llamar al robot

### Emociones Detectadas
- Neutral, Happy, Sad, Angry
- Fear, Surprise, Disgust
- Valence (-1 a +1) y Arousal (0 a 1)

### Intenciones Soportadas
- **GREETING/FAREWELL**: Saludos
- **NAVIGATE**: Ir a lugar
- **FETCH**: Traer objeto
- **FOLLOW**: Seguir persona
- **STOP**: Detener
- **HELP**: Ayuda

### Seguridad
```python
# Verificar antes de moverse
safe, reason = hri.get_safety_status()["safe_to_move"]
if not safe:
    print(f"Inseguro: {reason}")
    
# Parada de emergencia
hri.emergency_stop()

# Reset (solo si zona despejada)
hri.reset_emergency()
```

### Diálogo Multi-Turno
El sistema mantiene contexto entre turnos:
```
Usuario: "Tráeme un vaso"
Robot:  "¿Dónde está el vaso?"
Usuario: "En la cocina"
Robot:  "Entendido, voy a traer el vaso de la cocina"
```
        """,
        
        steps=[
            POTStep(
                id="detect_humans",
                name="Detectar humanos",
                description="Localizar personas en el entorno",
                step_type=StepType.LOG,
                tutorial_notes="""
Usar sistema de visión para detectar personas:
```python
persons = vision.detect_persons(frame)
# [{"id": 1, "position": [x,y,z], "distance": d}, ...]
```
                """,
            ),
            POTStep(
                id="update_safety",
                name="Actualizar seguridad",
                description="Establecer nivel según proximidad",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.hri import SafetyMonitor; s=SafetyMonitor(); print(s.to_dict())"',
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="init_hri",
                name="Inicializar HRI",
                description="Arrancar sistema de interacción",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.hri import HRISystem; h=HRISystem(); print(h.to_dict())"',
                timeout_seconds=15,
                capture_output=True,
            ),
            POTStep(
                id="listen_commands",
                name="Escuchar comandos",
                description="Activar escucha de voz/gestos",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
hri.start()
# Loop de procesamiento
while running:
    # Procesar audio
    if voice_input:
        response = hri.process_text_input(voice_input)
    # Procesar visual
    results = hri.process_visual_input(frame, persons)
```
                """,
            ),
            POTStep(
                id="process_intent",
                name="Procesar intención",
                description="Interpretar comando",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
from modules.humanoid.hri import IntentParser
parser = IntentParser()
intent = parser.parse("ve a la cocina")
print(intent.type, intent.slots)
```
                """,
            ),
            POTStep(
                id="execute_safe",
                name="Ejecutar de forma segura",
                description="Realizar acción respetando zonas",
                step_type=StepType.CHECK,
                tutorial_notes="""
Siempre verificar seguridad antes de actuar:
```python
safe, reason = safety.is_safe_to_move()
if safe:
    execute_action(action)
else:
    speak(f"No puedo moverme: {reason}")
```
                """,
            ),
            POTStep(
                id="log_interaction",
                name="Registrar interacción",
                description="Guardar log de la sesión",
                step_type=StepType.NOTIFY,
                notify_message="Sesión de interacción completada",
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_stop",
                name="Parada de emergencia",
                description="Detener robot de forma segura",
                step_type=StepType.LOG,
            ),
        ],
        
        has_rollback=True,
        tags=["hri", "voice", "gesture", "safety", "dialog"],
    )
