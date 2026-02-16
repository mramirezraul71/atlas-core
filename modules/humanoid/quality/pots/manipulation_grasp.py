"""
POT: Manipulation & Grasping Operations
========================================
Procedimientos para manipulación de objetos y control de manos.

Triggers:
- Solicitud de agarre
- Control de brazo
- Detección de objetos
- Liberación de objetos

Severidad: HIGH (manipulación física)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="manipulation_grasp",
        name="Manipulación y Grasping",
        description="""
Procedimiento para operaciones de manipulación:
1. Detectar objeto a manipular
2. Estimar pose del objeto
3. Planificar agarre
4. Ejecutar agarre
5. Verificar y confirmar
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["grasp_*", "arm_*", "hand_*", "manipulation_*"],
        trigger_keywords=["agarrar", "soltar", "mano", "brazo", "objeto", "pick", "place"],
        
        prerequisites=[
            "Brazos y manos calibrados",
            "Cámara de profundidad activa",
            "Espacio de trabajo despejado",
        ],
        required_services=["manipulation", "vision"],
        required_permissions=["movement", "manipulation"],
        
        objectives=[
            "Detectar y localizar objeto",
            "Generar candidatos de agarre",
            "Planificar trayectoria del brazo",
            "Ejecutar secuencia de agarre",
            "Verificar agarre exitoso",
        ],
        success_criteria="Objeto agarrado con fuerza estable",
        estimated_duration_minutes=5,
        
        tutorial_overview="""
## Guía de Manipulación ATLAS

### Arquitectura
```
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  ObjectPose   │────▶│ GraspPlanner  │────▶│ GraspExecutor │
│  Estimator    │     │  (antipodal)  │     │   (motion)    │
└───────────────┘     └───────────────┘     └───────────────┘
                              │
                              ▼
                      ┌───────────────┐
                      │     Hand      │
                      │  Controller   │
                      └───────────────┘
```

### Pipeline de Agarre
1. **Detectar**: Segmentar objeto del fondo
2. **Estimar pose**: Posición y orientación
3. **Planificar**: Generar candidatos antipodales
4. **Aproximar**: Mover brazo a posición
5. **Agarrar**: Cerrar dedos con control de fuerza
6. **Verificar**: Confirmar agarre estable
7. **Manipular**: Mover objeto a destino

### Tipos de Agarre
- **parallel**: Agarre con dos dedos opuestos
- **power**: Agarre de fuerza con toda la mano
- **precision**: Agarre de precisión con punta de dedos
- **pinch**: Agarre de pinza (pulgar + índice)

### Uso Básico
```python
from modules.humanoid.manipulation import (
    GraspPlanner, GraspExecutor, ObjectPoseEstimator
)

# Detectar objeto
estimator = ObjectPoseEstimator()
poses = estimator.estimate_from_pointcloud(object_cloud)

# Planificar agarre
planner = GraspPlanner()
grasps = planner.plan_grasp(object_cloud)

# Ejecutar mejor agarre
executor = GraspExecutor()
result = executor.execute(grasps[0])

if result.success:
    print("Objeto agarrado!")
```

### Control de Mano
```python
from modules.humanoid.manipulation import HandController

hand = HandController("right")

# Poses predefinidas
hand.set_pose("open")
hand.set_pose("pinch")
hand.set_pose("power")

# Control individual de dedos
hand.set_finger("index", [0.5, 0.5, 0.5, 0.0])

# Verificar agarre
if hand.is_grasping():
    print(f"Fuerza total: {hand.get_total_force()} N")
```

### Cinemática de Brazo
```python
from modules.humanoid.manipulation import ArmKinematics

arm = ArmKinematics()

# Forward kinematics
pos, rot = arm.forward_kinematics(joint_positions)

# Inverse kinematics
joints = arm.inverse_kinematics(target_position, target_rotation)

# Verificar alcance
if arm.is_reachable(target):
    print("Posición alcanzable")
```

### Manejo de Errores
- Si el agarre falla, intentar siguiente candidato
- Si no hay candidatos, reposicionar y re-escanear
- Monitorear fuerza durante transporte
- Soltar suavemente si se detecta pérdida
        """,
        
        steps=[
            POTStep(
                id="detect_object",
                name="Detectar objeto",
                description="Localizar objeto a manipular",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.manipulation import ObjectPoseEstimator; e=ObjectPoseEstimator(); print(e.to_dict())"',
                timeout_seconds=15,
                capture_output=True,
            ),
            POTStep(
                id="estimate_pose",
                name="Estimar pose",
                description="Calcular posición y orientación",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
poses = estimator.estimate_from_pointcloud(cloud)
if poses:
    target = poses[0]
    print(f"Objeto en: {target.position}")
```
                """,
            ),
            POTStep(
                id="plan_grasp",
                name="Planificar agarre",
                description="Generar candidatos de agarre",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.manipulation import GraspPlanner; p=GraspPlanner(); print(p.config)"',
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="move_arm",
                name="Mover brazo",
                description="Posicionar brazo para agarre",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
arm = ArmKinematics()
joints = arm.inverse_kinematics(approach_pos)
# Enviar a controlador de joints
```
                """,
            ),
            POTStep(
                id="execute_grasp",
                name="Ejecutar agarre",
                description="Cerrar gripper/mano",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
executor = GraspExecutor()
result = executor.execute(best_grasp)
print(f"Éxito: {result.success}, Fuerza: {result.force_applied}N")
```
                """,
            ),
            POTStep(
                id="verify_grasp",
                name="Verificar agarre",
                description="Confirmar agarre estable",
                step_type=StepType.CHECK,
                tutorial_notes="""
Verificar:
- Contacto en múltiples dedos
- Fuerza > 2N
- Objeto no desliza
                """,
            ),
            POTStep(
                id="log_result",
                name="Registrar resultado",
                description="Guardar resultado de manipulación",
                step_type=StepType.NOTIFY,
                notify_message="Operación de manipulación completada",
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_release",
                name="Soltar objeto",
                description="Liberar objeto de forma segura",
                step_type=StepType.LOG,
            ),
        ],
        
        has_rollback=True,
        tags=["manipulation", "grasp", "arm", "hand", "kinematics"],
    )
