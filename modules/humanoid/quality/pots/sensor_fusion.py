"""
POT: Sensor Fusion Operations
==============================
Procedimientos para fusión sensorial multi-modal.

Triggers:
- Calibración de sensores
- Actualización de fusión
- Detección de anomalías sensoriales
- Diagnóstico de sensores

Severidad: HIGH (datos críticos para control)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="sensor_fusion",
        name="Fusión Sensorial",
        description="""
Procedimiento para operaciones de fusión sensorial:
1. Verificar estado de sensores
2. Calibrar sensores si es necesario
3. Configurar filtros Kalman
4. Iniciar fusión multi-modal
5. Monitorear calidad de datos
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["sensor_*", "imu_*", "depth_*", "encoder_*"],
        trigger_keywords=["sensores", "fusión", "IMU", "depth", "calibrar", "kalman"],
        
        prerequisites=[
            "Hardware sensorial conectado",
            "Drivers de sensores instalados",
        ],
        required_services=["sensor_fusion"],
        required_permissions=["sensors"],
        
        objectives=[
            "Verificar conectividad de sensores",
            "Ejecutar calibración de IMU",
            "Calibrar sensores de fuerza",
            "Iniciar sistema de fusión",
            "Validar calidad de estimación",
        ],
        success_criteria="Fusión funcionando con confianza > 80%",
        estimated_duration_minutes=10,
        
        tutorial_overview="""
## Guía de Fusión Sensorial ATLAS

### Arquitectura
```
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
│   IMU   │  │  Depth  │  │Encoders │  │   F/T   │
│ Sensor  │  │ Camera  │  │         │  │ Sensor  │
└────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘
     │            │            │            │
     ▼            ▼            ▼            ▼
┌─────────────────────────────────────────────────┐
│              SENSOR FUSION ENGINE               │
│  ┌─────────┐  ┌─────────┐  ┌─────────────────┐  │
│  │ Kalman  │  │ Outlier │  │ State Estimate  │  │
│  │ Filter  │  │ Reject  │  │    (Fused)      │  │
│  └─────────┘  └─────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────┘
                      │
                      ▼
               ┌──────────────┐
               │ FUSED STATE  │
               │ pos, vel,    │
               │ orient, etc  │
               └──────────────┘
```

### Sensores Soportados
1. **IMU** - Aceleración, giroscopio, magnetómetro
2. **Depth Camera** - Profundidad, RGB, point cloud
3. **Encoders** - Posición/velocidad de joints
4. **Force/Torque** - Fuerzas de contacto

### Filtros Disponibles
- **KalmanFilter** - Sistemas lineales
- **ExtendedKalmanFilter (EKF)** - No lineales
- **LowPassFilter** - Suavizado simple

### Uso Básico
```python
from modules.humanoid.sensors import SensorFusion, IMUSensor

# Crear sistema
fusion = SensorFusion()

# Añadir sensores
fusion.add_sensor("imu", IMUSensor())
fusion.add_sensor("encoders", MotorEncoder(num_joints=20))

# Iniciar loop
fusion.start()

# Obtener estado
state = fusion.get_state()
print(state.position, state.orientation)
```

### Calibración
```python
# Calibrar IMU (robot quieto)
imu = IMUSensor()
samples = [imu.process(raw) for raw in collect_static_samples()]
imu.calibrate(samples)

# Calibrar F/T (sin carga)
ft = ForceTorqueSensor()
samples = [ft.process(raw) for raw in collect_unloaded_samples()]
ft.calibrate(samples)
```

### Diagnóstico
```python
stats = fusion.get_stats()
print(f"Active sensors: {stats['active_sensors']}/{len(stats['sensors'])}")
print(f"Confidence: {stats['confidence']:.1%}")
print(f"Outliers rejected: {stats['outliers_rejected']}")
```
        """,
        
        steps=[
            POTStep(
                id="check_sensors",
                name="Verificar sensores",
                description="Comprobar conectividad de sensores",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.sensors import SensorFusion; f=SensorFusion(); print(f.get_stats())"',
                timeout_seconds=15,
                capture_output=True,
            ),
            POTStep(
                id="calibrate_imu",
                name="Calibrar IMU",
                description="Ejecutar calibración de IMU",
                step_type=StepType.LOG,
                tutorial_notes="""
Calibración de IMU:
1. Robot completamente quieto
2. Recolectar 50-100 muestras
3. Calcular bias de giroscopio y acelerómetro

```python
imu = IMUSensor()
samples = []
for _ in range(100):
    raw = get_imu_raw()
    samples.append(imu.process(raw))
    time.sleep(0.01)
result = imu.calibrate(samples)
```
                """,
            ),
            POTStep(
                id="setup_kalman",
                name="Configurar Kalman",
                description="Inicializar filtro EKF",
                step_type=StepType.COMMAND,
                command='python -c "from modules.humanoid.sensors import ExtendedKalmanFilter; ekf=ExtendedKalmanFilter(15,6); print(\'EKF initialized:\', ekf.n, \'states\')"',
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="start_fusion",
                name="Iniciar fusión",
                description="Arrancar el loop de fusión",
                step_type=StepType.LOG,
                tutorial_notes="""
```python
fusion = SensorFusion()
fusion.add_sensor("imu", imu)
fusion.add_sensor("depth", depth_cam)
fusion.start()  # Loop en background
```
                """,
            ),
            POTStep(
                id="validate_fusion",
                name="Validar fusión",
                description="Verificar calidad de estimación",
                step_type=StepType.CHECK,
                tutorial_notes="""
Verificar:
- Confidence > 80%
- No hay sensores en timeout
- Outliers < 5% de muestras
                """,
            ),
            POTStep(
                id="log_results",
                name="Registrar resultados",
                description="Guardar estado del sistema",
                step_type=StepType.NOTIFY,
                notify_message="Fusión sensorial operativa",
            ),
        ],
        
        has_rollback=False,
        tags=["sensors", "fusion", "imu", "kalman", "calibration"],
    )
