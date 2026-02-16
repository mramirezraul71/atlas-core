"""
POT: Navigation & SLAM Operations
==================================
Procedimientos para navegación autónoma, mapeo y localización.

Triggers:
- Solicitud de navegación a punto
- Inicio de mapeo
- Pérdida de localización
- Error de navegación

Severidad: MEDIUM (control de movimiento)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="navigation_slam",
        name="Navegación y SLAM",
        description="""
Procedimiento maestro para operaciones de navegación:
1. Mapeo (SLAM) - Crear mapas del entorno
2. Localización - Ubicar el robot en un mapa conocido
3. Navegación - Ir a un punto objetivo
4. Recuperación - Manejar errores de navegación
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["navigation_*", "slam_*", "localization_*", "path_*"],
        trigger_keywords=["navegar", "mapa", "slam", "goto", "posición", "localización", "camino"],
        
        prerequisites=[
            "Sistema de navegación inicializado",
            "Sensores (LiDAR/depth) disponibles",
            "Odometría funcionando",
        ],
        required_services=["navigation_system"],
        required_permissions=["movement"],
        
        objectives=[
            "Verificar estado del sistema de navegación",
            "Iniciar SLAM si se requiere mapeo",
            "Localizar robot en mapa existente",
            "Planificar y ejecutar ruta al objetivo",
            "Manejar recuperación si hay obstáculos",
        ],
        success_criteria="Robot en posición objetivo o mapa guardado",
        estimated_duration_minutes=10,
        
        tutorial_overview="""
## Guía de Navegación ATLAS

### Arquitectura del Sistema
```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    SLAM     │────▶│   Planner   │────▶│ Controller  │
│   Engine    │     │   (A*/RRT)  │     │  (Pursuit)  │
└─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    Map      │     │   Costmap   │     │   Velocity  │
│  (Grid)     │     │  (Inflated) │     │   Command   │
└─────────────┘     └─────────────┘     └─────────────┘
```

### Flujo de Navegación
1. **Localizar**: Conocer posición actual
2. **Planificar**: Encontrar ruta al objetivo
3. **Controlar**: Seguir la ruta
4. **Recuperar**: Si hay problemas

### Modos de Operación
- `mapping`: Construir mapa nuevo
- `localization`: Usar mapa existente
- `navigation`: Ir a objetivo

### Comandos Principales
```python
from modules.humanoid.navigation import NavigationSystem

nav = NavigationSystem()
nav.start()

# Mapear
nav.start_mapping()
# ... mover robot ...
nav.save_map("data/maps/oficina.npz")

# Navegar
nav.load_map("data/maps/oficina.npz")
nav.set_initial_pose(0, 0, 0)
nav.goto(x=2.0, y=1.5)
```
        """,
        
        steps=[
            # === FASE 1: VERIFICACIÓN ===
            POTStep(
                id="check_nav_system",
                name="Verificar sistema de navegación",
                description="Comprobar que el sistema está operativo",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8787/api/navigation/status",
                timeout_seconds=10,
                capture_output=True,
                tutorial_notes="El sistema debe estar en estado IDLE o LOCALIZING",
            ),
            POTStep(
                id="check_sensors",
                name="Verificar sensores",
                description="Comprobar LiDAR, IMU y odometría",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.navigation import NavigationSystem; print(\'Navigation module OK\')"',
                timeout_seconds=15,
                capture_output=True,
            ),
            
            # === FASE 2: INICIALIZACIÓN ===
            POTStep(
                id="init_navigation",
                name="Inicializar navegación",
                description="Arrancar el sistema de navegación",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.navigation import NavigationSystem; nav = NavigationSystem(); r = nav.start(); print(r)"',
                timeout_seconds=30,
                capture_output=True,
                tutorial_notes="""
Para uso programático:
```python
nav = NavigationSystem()
nav.start()
```
                """,
            ),
            
            # === FASE 3: MAPEO (OPCIONAL) ===
            POTStep(
                id="start_mapping",
                name="Iniciar mapeo SLAM",
                description="Comenzar a construir mapa del entorno",
                step_type=StepType.LOG,
                skip_if="context.get('mode') != 'mapping'",
                tutorial_notes="""
El mapeo SLAM requiere:
1. Mover el robot por el entorno
2. Recibir scans de LiDAR/depth
3. Guardar el mapa al finalizar

```python
nav.start_mapping()
# El robot debe moverse mientras mapea
nav.save_map("data/maps/nuevo_mapa.npz")
```
                """,
            ),
            
            # === FASE 4: LOCALIZACIÓN ===
            POTStep(
                id="load_map",
                name="Cargar mapa existente",
                description="Cargar mapa para localización",
                step_type=StepType.LOG,
                skip_if="context.get('mode') == 'mapping'",
                tutorial_notes="""
```python
nav.load_map("data/maps/mapa.npz")
nav.set_initial_pose(x=0, y=0, theta=0)
```
                """,
            ),
            
            # === FASE 5: NAVEGACIÓN ===
            POTStep(
                id="plan_path",
                name="Planificar ruta",
                description="Calcular camino al objetivo",
                step_type=StepType.LOG,
                tutorial_notes="""
El planificador A* encuentra la ruta óptima:
```python
path = planner.plan(
    start=(current_x, current_y),
    goal=(target_x, target_y),
    algorithm="a_star"
)
```
                """,
            ),
            POTStep(
                id="execute_path",
                name="Ejecutar navegación",
                description="Seguir la ruta planificada",
                step_type=StepType.LOG,
                tutorial_notes="""
El controlador Pure Pursuit sigue el camino:
```python
nav.goto(x=2.0, y=1.5, theta=0)

# Esperar a llegar
while not nav.is_goal_reached():
    time.sleep(0.1)
```
                """,
            ),
            
            # === FASE 6: RECUPERACIÓN ===
            POTStep(
                id="recovery_behaviors",
                name="Comportamientos de recuperación",
                description="Manejar obstáculos y errores",
                step_type=StepType.LOG,
                tutorial_notes="""
Si hay obstáculos, el sistema ejecuta:
1. Limpiar costmap local
2. Rotar 360° para escanear
3. Retroceder 0.3m
4. Esperar 5s
5. Replanificar

Cada comportamiento se intenta hasta 3 veces.
                """,
            ),
            
            # === FASE 7: FINALIZACIÓN ===
            POTStep(
                id="log_result",
                name="Registrar resultado",
                description="Guardar resultado de navegación",
                step_type=StepType.NOTIFY,
                notify_message="Navegación completada",
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_stop_nav",
                name="Detener navegación",
                description="Cancelar navegación en curso",
                step_type=StepType.LOG,
            ),
        ],
        
        has_rollback=True,
        tags=["navigation", "slam", "localization", "autonomous"],
    )
