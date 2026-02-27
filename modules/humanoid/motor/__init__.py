"""
Motor Control Module: Control de movimiento y trayectorias.

Incluye:
- TrajectoryPlanner: Planificacion de trayectorias
- MotorController: Control de bajo nivel PID/impedancia
- MotorInterface: Interfaz de alto nivel para comandos
"""
from .trajectory_planner import (
    TrajectoryPlanner,
    Trajectory,
    Waypoint,
    Pose3D,
    JointState,
    TrajectoryType,
    Obstacle,
)
from .motor_controller import (
    MotorController,
    PIDController,
    ImpedanceController,
    PIDGains,
    ImpedanceParams,
    JointConfig,
    JointCommand,
    JointFeedback,
    ControlMode,
)
from .motor_interface import (
    MotorInterface,
    HighLevelCommand,
    ExecutionResult,
    CommandType,
    ExecutionStatus,
)

__all__ = [
    # Trajectory Planner
    "TrajectoryPlanner",
    "Trajectory",
    "Waypoint",
    "Pose3D",
    "JointState",
    "TrajectoryType",
    "Obstacle",
    # Motor Controller
    "MotorController",
    "PIDController",
    "ImpedanceController",
    "PIDGains",
    "ImpedanceParams",
    "JointConfig",
    "JointCommand",
    "JointFeedback",
    "ControlMode",
    # Motor Interface
    "MotorInterface",
    "HighLevelCommand",
    "ExecutionResult",
    "CommandType",
    "ExecutionStatus",
]
