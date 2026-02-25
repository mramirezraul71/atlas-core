"""
ATLAS Sensor Fusion Module
===========================
Sistema de fusión sensorial multi-modal.

Componentes:
- SensorFusion: Fusión de múltiples sensores
- KalmanFilter: Filtros de Kalman (EKF, UKF)
- IMU: Procesamiento de IMU
- DepthCamera: Procesamiento de cámaras de profundidad
- Encoders: Encoders de motores
- ForceTorque: Sensores de fuerza/torque

Uso:
    from modules.humanoid.sensors import SensorFusion

    fusion = SensorFusion()
    fusion.add_sensor("imu", IMUSensor())
    fusion.add_sensor("depth", DepthCamera())
    state = fusion.update()
"""
from .depth_camera import DepthCamera, DepthFrame
from .encoders import EncoderData, MotorEncoder
from .force_torque import ForceTorqueSensor, WrenchData
from .imu_sensor import IMUData, IMUSensor
from .kalman_filter import ExtendedKalmanFilter, KalmanFilter
from .sensor_fusion import FusedState, FusionConfig, SensorFusion

__all__ = [
    "SensorFusion",
    "FusionConfig",
    "FusedState",
    "KalmanFilter",
    "ExtendedKalmanFilter",
    "IMUSensor",
    "IMUData",
    "DepthCamera",
    "DepthFrame",
    "MotorEncoder",
    "EncoderData",
    "ForceTorqueSensor",
    "WrenchData",
]
