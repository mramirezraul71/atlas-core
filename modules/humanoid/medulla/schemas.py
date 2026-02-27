"""
Schemas de mensajes para la Médula Atlas.

Define estructuras de datos para comunicación entre sensores, actuadores y módulos cognitivos.
Equivalente a Protobuf pero usando dataclasses para simplicidad inicial.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Union
from enum import Enum


class MessageType(Enum):
    """Tipos de mensajes en el bus."""
    SENSOR_VISION = "sensor.vision"
    SENSOR_AUDIO = "sensor.audio"
    SENSOR_IMU = "sensor.imu"
    SENSOR_FORCE_TORQUE = "sensor.ft"
    SENSOR_ENCODER = "sensor.encoder"
    ACTUATOR_ARM = "actuator.arm"
    ACTUATOR_GRIPPER = "actuator.gripper"
    ACTUATOR_LOCOMOTION = "actuator.locomotion"
    ACTUATOR_NECK = "actuator.neck"
    COGNITIVE_PERCEPTION = "cognitive.perception"
    COGNITIVE_DECISION = "cognitive.decision"
    COGNITIVE_GOAL = "cognitive.goal"
    SYSTEM_VITALS = "system.vitals"
    SYSTEM_ALERT = "system.alert"


@dataclass
class BoundingBox:
    """Bounding box para detecciones."""
    x: float
    y: float
    width: float
    height: float
    
    def center(self) -> tuple:
        return (self.x + self.width / 2, self.y + self.height / 2)
    
    def area(self) -> float:
        return self.width * self.height


@dataclass
class Pose3D:
    """Pose 3D (posición + orientación)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0  # Quaternion
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    
    def position(self) -> tuple:
        return (self.x, self.y, self.z)
    
    def orientation(self) -> tuple:
        return (self.qx, self.qy, self.qz, self.qw)


@dataclass
class Detection:
    """Detección de objeto en imagen."""
    class_name: str
    confidence: float
    bbox: BoundingBox
    pose: Optional[Pose3D] = None
    object_id: Optional[str] = None
    
    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class VisionFrame:
    """Frame de visión con detecciones."""
    camera_id: str
    width: int
    height: int
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    rgb_data: Optional[bytes] = None  # JPEG comprimido
    depth_data: Optional[bytes] = None  # uint16 raw
    detections: List[Detection] = field(default_factory=list)
    
    def detection_count(self) -> int:
        return len(self.detections)
    
    def get_detection_by_class(self, class_name: str) -> List[Detection]:
        return [d for d in self.detections if d.class_name == class_name]


@dataclass
class IMUReading:
    """Lectura de sensor IMU."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    # Acelerómetro (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    # Giroscopio (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # Magnetómetro (μT)
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    # Orientación fusionada (quaternion)
    orientation: Optional[Pose3D] = None
    temperature: float = 25.0


@dataclass
class ForceTorqueReading:
    """Lectura de sensor de fuerza/torque."""
    joint_id: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    # Fuerzas (N)
    force_x: float = 0.0
    force_y: float = 0.0
    force_z: float = 0.0
    # Torques (Nm)
    torque_x: float = 0.0
    torque_y: float = 0.0
    torque_z: float = 0.0
    
    def force_magnitude(self) -> float:
        return (self.force_x**2 + self.force_y**2 + self.force_z**2) ** 0.5
    
    def torque_magnitude(self) -> float:
        return (self.torque_x**2 + self.torque_y**2 + self.torque_z**2) ** 0.5


@dataclass
class EncoderReading:
    """Lectura de encoder de articulación."""
    joint_id: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    position: float = 0.0  # rad
    velocity: float = 0.0  # rad/s
    effort: float = 0.0  # Nm (torque medido)
    temperature: float = 25.0


@dataclass
class SensorReading:
    """Wrapper genérico para lecturas de sensores."""
    sensor_id: str
    sensor_type: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    data: Union[VisionFrame, IMUReading, ForceTorqueReading, EncoderReading, dict] = field(default_factory=dict)
    
    def to_bytes(self) -> bytes:
        """Serializa a bytes para transmisión."""
        return json.dumps(asdict(self), default=str).encode('utf-8')
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'SensorReading':
        """Deserializa desde bytes."""
        d = json.loads(data.decode('utf-8'))
        return cls(**d)


# === Comandos de Actuadores ===

@dataclass
class JointPositionCommand:
    """Comando de posición para articulación."""
    joint_id: str
    position: float  # rad
    velocity_limit: float = 1.0  # rad/s
    acceleration_limit: float = 2.0  # rad/s²


@dataclass
class JointVelocityCommand:
    """Comando de velocidad para articulación."""
    joint_id: str
    velocity: float  # rad/s
    acceleration_limit: float = 2.0  # rad/s²


@dataclass
class JointTorqueCommand:
    """Comando de torque para articulación."""
    joint_id: str
    torque: float  # Nm
    duration_ms: int = 100  # Duración máxima


@dataclass
class GripperCommand:
    """Comando para gripper/mano."""
    gripper_id: str
    action: str  # "open", "close", "position", "force"
    position: float = 0.0  # 0=cerrado, 1=abierto
    force_limit: float = 10.0  # N
    speed: float = 0.5  # 0-1


@dataclass
class MotorCommand:
    """Wrapper genérico para comandos de actuadores."""
    actuator_id: str
    command_type: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    command: Union[JointPositionCommand, JointVelocityCommand, JointTorqueCommand, GripperCommand, dict] = field(default_factory=dict)
    priority: int = 0  # 0=normal, >0=alta prioridad
    
    def to_bytes(self) -> bytes:
        """Serializa a bytes para transmisión."""
        return json.dumps(asdict(self), default=str).encode('utf-8')
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'MotorCommand':
        """Deserializa desde bytes."""
        d = json.loads(data.decode('utf-8'))
        return cls(**d)


# === Mensajes Cognitivos ===

@dataclass
class ObjectState:
    """Estado de un objeto en el mundo."""
    object_id: str
    class_name: str
    pose: Pose3D
    confidence: float = 1.0
    velocity: Optional[Pose3D] = None  # Velocidad lineal/angular
    last_seen_ns: int = field(default_factory=lambda: time.time_ns())
    properties: Dict[str, Any] = field(default_factory=dict)


@dataclass
class HumanState:
    """Estado de un humano detectado."""
    human_id: str
    pose: Pose3D  # Posición del torso
    face_detected: bool = False
    face_embedding: Optional[List[float]] = None
    skeleton: Optional[Dict[str, Pose3D]] = None  # Keypoints
    gaze_direction: Optional[Pose3D] = None
    speaking: bool = False
    last_seen_ns: int = field(default_factory=lambda: time.time_ns())


@dataclass
class WorldState:
    """Estado del mundo percibido."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    robot_pose: Pose3D = field(default_factory=Pose3D)
    objects: List[ObjectState] = field(default_factory=list)
    humans: List[HumanState] = field(default_factory=list)
    internal_state: Dict[str, float] = field(default_factory=dict)  # battery, temp, etc.
    
    def get_object(self, object_id: str) -> Optional[ObjectState]:
        for obj in self.objects:
            if obj.object_id == object_id:
                return obj
        return None
    
    def get_objects_by_class(self, class_name: str) -> List[ObjectState]:
        return [o for o in self.objects if o.class_name == class_name]
    
    def nearest_human(self) -> Optional[HumanState]:
        if not self.humans:
            return None
        # Distancia desde robot_pose
        def dist(h):
            return ((h.pose.x - self.robot_pose.x)**2 + 
                    (h.pose.y - self.robot_pose.y)**2) ** 0.5
        return min(self.humans, key=dist)
    
    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class ActionDecision:
    """Decisión de acción del sistema cognitivo."""
    action_id: str
    action_type: str  # "move", "grasp", "speak", "wait", "look"
    parameters: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    reasoning: str = ""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    priority: int = 0
    
    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class GoalUpdate:
    """Actualización de objetivo."""
    goal_id: str
    goal_type: str  # "navigate", "fetch", "place", "speak", "observe"
    target: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    priority: int = 0
    deadline_ns: Optional[int] = None
    status: str = "pending"  # pending, active, completed, failed, cancelled
    progress: float = 0.0  # 0-1
    
    def to_dict(self) -> dict:
        return asdict(self)


# === Mensajes de Sistema ===

@dataclass
class VitalsStatus:
    """Estado de signos vitales del robot."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    battery_percent: float = 100.0
    cpu_temp: float = 45.0
    gpu_temp: float = 50.0
    motor_temps: Dict[str, float] = field(default_factory=dict)
    memory_used_mb: int = 0
    disk_used_percent: float = 0.0
    network_connected: bool = True
    emergency_stop: bool = False
    mode: str = "normal"  # normal, safe, emergency, learning
    
    def is_critical(self) -> bool:
        return (
            self.battery_percent < 10 or
            self.cpu_temp > 85 or
            self.emergency_stop or
            any(t > 70 for t in self.motor_temps.values())
        )


@dataclass
class SystemAlert:
    """Alerta del sistema."""
    alert_id: str
    level: str  # "info", "warning", "error", "critical", "emergency"
    source: str
    message: str
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    data: Dict[str, Any] = field(default_factory=dict)
    requires_ack: bool = False
    acknowledged: bool = False
