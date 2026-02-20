# Atlas ROS 2 Spine — Columna Vertebral

## Arquitectura

```
┌─────────────────────────────────────────────────────────────────┐
│                    LAYER 5: SUPERVISOR                          │
│  atlas_supervisor ←→ atlas_brain_bridge                         │
│  (Governance, Heartbeat, Task Dispatch, AI Reasoning)           │
│  Bridges: PUSH :8791 | ROBOT :8002 | ANS | Memory | Governance │
├─────────────────────────────────────────────────────────────────┤
│                    LAYER 4: PLANNING                            │
│  atlas_planning                                                 │
│  task_planner (AI goal decomposition)                           │
│  path_planner (Nav2-compatible waypoint navigation)             │
├─────────────────────────────────────────────────────────────────┤
│                    LAYER 3: PERCEPTION                          │
│  atlas_perception                                               │
│  vision_node (cameras RGB/depth)                                │
│  object_detector (YOLOv8, reuses existing yolov8n.pt)           │
│  slam_node (localization + mapping placeholder)                 │
├─────────────────────────────────────────────────────────────────┤
│                    LAYER 2: CONTROL                             │
│  atlas_motor_control                                            │
│  balance_controller (PID, CoP, ZMP — IHMC-inspired)             │
│  gait_generator (stand, walk, trot — bipedal locomotion)        │
│  joint_commander (command merge + hardware dispatch)             │
├─────────────────────────────────────────────────────────────────┤
│                    LAYER 1: HARDWARE                            │
│  atlas_sensors                                                  │
│  imu_publisher (orientation, angular vel, accel)                │
│  joint_state_publisher (30 DOF encoder readout)                 │
│  force_torque_publisher (foot GRF for balance)                  │
├─────────────────────────────────────────────────────────────────┤
│                    SHARED                                       │
│  atlas_interfaces (custom .msg, .srv, .action)                  │
│  atlas_bringup (launch files, global config)                    │
└─────────────────────────────────────────────────────────────────┘
```

## Paquetes

| Paquete | Capa | Descripcion |
|---------|------|-------------|
| `atlas_bringup` | Launch | Archivos de lanzamiento y configuracion global |
| `atlas_interfaces` | Shared | Mensajes, servicios y acciones ROS 2 custom |
| `atlas_sensors` | L1 Hardware | Drivers de sensores: IMU, joints, force-torque |
| `atlas_motor_control` | L2 Control | Balance (IHMC), marcha bipeda, despacho de comandos |
| `atlas_perception` | L3 Percepcion | Vision (YOLO), profundidad, SLAM |
| `atlas_planning` | L4 Planificacion | Planificador de tareas (IA) y rutas (Nav2) |
| `atlas_supervisor` | L5 Supervisor | Orquestador principal, gobernanza, heartbeat |
| `atlas_brain_bridge` | L5 Cerebro | Puente a IA (Bedrock/Anthropic), memoria, ANS |

## Relacion con Atlas existente

Esta estructura ROS 2 **NO reemplaza** los modulos existentes. Es una capa de middleware que:

1. **Conecta** los modulos existentes (`brain/`, `modules/humanoid/`, `autonomous/`) como nodos ROS 2
2. **Agrega** capacidades de robotica real: control de motores, balance, locomocion, SLAM
3. **Preserva** toda la logica de IA, gobernanza, memoria y orquestacion existente via HTTP bridges

### Puentes a sistemas existentes

```
ROS 2 Spine                    Atlas Existente
─────────────                  ───────────────
atlas_supervisor       ←→      atlas_adapter (PUSH :8791)
atlas_brain_bridge     ←→      brain/learning/ai_consultant.py (Bedrock)
                       ←→      modules/humanoid/orchestrator/
                       ←→      modules/humanoid/memory_engine/
                       ←→      modules/humanoid/governance/
                       ←→      modules/humanoid/ans/
atlas_perception       ←→      modules/humanoid/vision/
atlas_planning         ←→      modules/humanoid/navigation/
atlas_motor_control    ←→      modules/humanoid/motor/
atlas_sensors          ←→      modules/humanoid/sensors/
```

## Referencias de arquitectura

- **ROS 2 Humble/Iron** — Middleware estandar de robotica
- **IHMC Open Robotics Software** — Control de humanoides (balance, impulso, ZMP)
- **RoboParty / Roboto Origin** — Stack full-stack open-source para bipedos
- **robotpkg** — Paquetes de software robotico comunitarios
- **Nav2** — Navegacion autonoma para ROS 2

## Setup rapido

```bash
# 1. Instalar ROS 2 (Humble o Iron)
# Ver: https://docs.ros.org/en/humble/Installation.html

# 2. Crear workspace
cd C:\ATLAS_PUSH\ros2_ws
colcon build --symlink-install

# 3. Source
source install/setup.bash   # Linux
# o en Windows:
call install\setup.bat

# 4. Lanzar todo (simulacion)
ros2 launch atlas_bringup atlas_full.launch.py sim:=true

# 5. Lanzar con hardware real
ros2 launch atlas_bringup atlas_full.launch.py sim:=false
```

## Topics principales

| Topic | Tipo | Descripcion |
|-------|------|-------------|
| `/atlas/imu/data` | sensor_msgs/Imu | Datos IMU |
| `/atlas/joint_states` | sensor_msgs/JointState | Estado de articulaciones (30 DOF) |
| `/atlas/ft/left_foot` | geometry_msgs/WrenchStamped | Fuerza pie izquierdo |
| `/atlas/ft/right_foot` | geometry_msgs/WrenchStamped | Fuerza pie derecho |
| `/atlas/joint_commands` | sensor_msgs/JointState | Comandos de balance |
| `/atlas/gait/trajectory` | sensor_msgs/JointState | Trayectoria de marcha |
| `/atlas/gait/command` | std_msgs/String | Comando de marcha (walk/stand/stop) |
| `/atlas/hw/joint_target` | sensor_msgs/JointState | Comando final a hardware |
| `/atlas/camera/head/rgb` | sensor_msgs/Image | Camara RGB cabeza |
| `/atlas/camera/chest/depth` | sensor_msgs/Image | Camara profundidad pecho |
| `/atlas/perception/detections` | std_msgs/String | Detecciones YOLO (JSON) |
| `/atlas/slam/pose` | geometry_msgs/PoseStamped | Pose del robot (SLAM) |
| `/atlas/task/goal` | std_msgs/String | Meta de tarea (JSON) |
| `/atlas/task/plan` | std_msgs/String | Plan generado (JSON) |
| `/atlas/brain/query` | std_msgs/String | Query al cerebro IA |
| `/atlas/brain/response` | std_msgs/String | Respuesta del cerebro IA |
| `/atlas/supervisor/heartbeat` | std_msgs/String | Heartbeat del supervisor |
| `/atlas/supervisor/governance` | std_msgs/String | Cambios de gobernanza |

## DOF del humanoide (30 articulaciones)

```
Pierna izquierda (6):  l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee, l_ankle_pitch, l_ankle_roll
Pierna derecha (6):    r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee, r_ankle_pitch, r_ankle_roll
Torso (2):             torso_yaw, torso_pitch
Brazo izquierdo (7):   l_shoulder_pitch/roll/yaw, l_elbow, l_wrist_yaw/roll/pitch
Brazo derecho (7):     r_shoulder_pitch/roll/yaw, r_elbow, r_wrist_yaw/roll/pitch
Cabeza (2):            head_yaw, head_pitch
```
