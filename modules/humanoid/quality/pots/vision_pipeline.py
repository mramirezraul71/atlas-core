"""
POT: Vision Pipeline Operations
================================
Procedimientos para el pipeline de visión por computadora.

Triggers:
- Diagnóstico de cámaras
- Calibración de visión
- Detección de objetos/personas
- Procesamiento de imágenes

Severidad: HIGH (percepción crítica)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="vision_pipeline",
        name="Pipeline de Visión",
        description="""
Procedimiento para operaciones de visión:
1. Verificar estado de cámaras
2. Calibrar si es necesario
3. Procesar streams de video
4. Ejecutar detecciones
5. Publicar resultados
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["vision_*", "camera_*", "detection_*"],
        trigger_keywords=["visión", "cámara", "imagen", "detección", "objeto", "persona"],
        
        prerequisites=[
            "Cámaras conectadas y alimentadas",
            "Drivers de cámara instalados",
            "Modelos de detección disponibles",
        ],
        required_services=["vision"],
        required_permissions=["camera"],
        
        objectives=[
            "Verificar funcionamiento de cámaras",
            "Asegurar calibración correcta",
            "Procesar frames en tiempo real",
            "Detectar objetos y personas",
            "Estimar poses y profundidad",
        ],
        success_criteria="Pipeline procesando a >15 FPS con detecciones correctas",
        estimated_duration_minutes=5,
        
        tutorial_overview="""
## Guía del Pipeline de Visión ATLAS

### Arquitectura
```
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  RGB Camera │  │Depth Camera │  │   LiDAR     │
│  (640x480)  │  │  (D435i)    │  │ (optional)  │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                 │
       ▼                ▼                 ▼
┌─────────────────────────────────────────────────┐
│              FRAME ACQUISITION                   │
│         (sync, timestamping, buffering)          │
└───────────────────────┬─────────────────────────┘
                        │
       ┌────────────────┼────────────────┐
       ▼                ▼                ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  Object     │  │   Person    │  │    Face     │
│  Detection  │  │  Detection  │  │  Detection  │
│  (YOLO/SSD) │  │  (Pose Est) │  │  (landmarks)│
└─────────────┘  └─────────────┘  └─────────────┘
       │                │                │
       └────────────────┼────────────────┘
                        ▼
┌─────────────────────────────────────────────────┐
│              FUSION & TRACKING                   │
│         (multi-object, temporal)                 │
└───────────────────────┬─────────────────────────┘
                        ▼
┌─────────────────────────────────────────────────┐
│              RESULTS PUBLISHER                   │
│         (topics, callbacks, storage)             │
└─────────────────────────────────────────────────┘
```

### Cámaras Soportadas
- **Intel RealSense D435i**: RGB + Depth + IMU
- **Azure Kinect**: RGB + ToF Depth
- **USB Webcam**: RGB básico
- **IP Cameras**: RTSP streams

### Modelos de Detección
- **YOLOv8**: Objetos generales (80+ clases)
- **MobileNet-SSD**: Rápido, edge devices
- **MediaPipe**: Pose, hands, face
- **Custom**: Modelos entrenados

### Uso Básico
```python
from nexus.atlas_nexus_robot.backend.camera import CameraManager, CameraMode

# Crear manager
manager = CameraManager()

# Listar cámaras
cameras = manager.list_cameras()

# Iniciar cámara
manager.start_camera(0, mode=CameraMode.RGBD)

# Obtener frame
frame = manager.get_frame(0)
rgb = frame.rgb
depth = frame.depth

# Detección
detections = detector.detect(rgb)
for det in detections:
    print(f"{det.class_name}: {det.confidence:.2f}")
```

### Calibración
```python
# Calibración intrínseca
intrinsics = calibrate_intrinsics(checkerboard_images)

# Calibración estéreo
stereo_params = calibrate_stereo(left_images, right_images)

# Calibración eye-to-hand (robot)
transform = calibrate_eye_to_hand(robot_poses, marker_poses)
```

### Procesamiento de Depth
```python
from modules.humanoid.sensors import DepthCamera

depth_cam = DepthCamera()
frame = depth_cam.process(depth_data, color_data)

# Point cloud
points = depth_cam.depth_to_pointcloud(frame)

# Detectar obstáculos
obstacles = depth_cam.detect_obstacles(frame)

# Detectar suelo
floor = depth_cam.detect_floor(frame)
```

### Tracking
```python
# Multi-object tracking
tracker = MultiObjectTracker()
tracks = tracker.update(detections)

for track in tracks:
    print(f"ID {track.id}: {track.position}")
```

### Diagnóstico de Problemas
1. **Cámara no detectada**:
   - Verificar conexión USB/Ethernet
   - Verificar drivers: `lsusb` / Device Manager
   - Reiniciar servicio de cámara

2. **Frames negros**:
   - Verificar exposición/ganancia
   - Verificar iluminación
   - Verificar cable de video

3. **Baja FPS**:
   - Reducir resolución
   - Usar GPU para inferencia
   - Optimizar pipeline

4. **Detecciones erróneas**:
   - Ajustar umbral de confianza
   - Mejorar iluminación
   - Re-entrenar modelo si necesario
        """,
        
        steps=[
            POTStep(
                id="check_cameras",
                name="Verificar cámaras",
                description="Comprobar estado de todas las cámaras",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8787/api/vision/camera_list",
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="Lista todas las cámaras detectadas y su estado",
            ),
            POTStep(
                id="test_capture",
                name="Probar captura",
                description="Capturar frame de prueba",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8787/api/vision/capture_test",
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="check_calibration",
                name="Verificar calibración",
                description="Comprobar calibración de cámara",
                step_type=StepType.LOG,
                tutorial_notes="""
Verificar que los intrínsecos están cargados:
```python
intrinsics = camera.get_intrinsics()
# fx, fy, cx, cy deben tener valores razonables
```
                """,
            ),
            POTStep(
                id="start_pipeline",
                name="Iniciar pipeline",
                description="Arrancar procesamiento de video",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
# Iniciar pipeline de visión
vision_system.start()
# Verificar FPS
print(f"FPS: {vision_system.get_fps()}")
```
                """,
            ),
            POTStep(
                id="test_detection",
                name="Probar detección",
                description="Verificar detección de objetos",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
detections = detector.detect(frame)
print(f"Detectados: {len(detections)} objetos")
for d in detections:
    print(f"  {d.class_name}: {d.confidence:.2f}")
```
                """,
            ),
            POTStep(
                id="verify_performance",
                name="Verificar rendimiento",
                description="Comprobar FPS y latencia",
                step_type=StepType.CHECK,
                tutorial_notes="""
Métricas objetivo:
- FPS >= 15
- Latencia < 100ms
- Uso GPU < 80%
                """,
            ),
            POTStep(
                id="log_status",
                name="Registrar estado",
                description="Guardar estado del pipeline",
                step_type=StepType.NOTIFY,
                notify_message="Pipeline de visión operativo",
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_stop_pipeline",
                name="Detener pipeline",
                description="Parar procesamiento de video",
                step_type=StepType.LOG,
            ),
        ],
        
        has_rollback=True,
        tags=["vision", "camera", "detection", "depth", "tracking"],
    )
