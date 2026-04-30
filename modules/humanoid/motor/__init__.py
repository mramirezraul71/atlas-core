"""
Motor Control Module: Control de movimiento y trayectorias.

Incluye:
- TrajectoryPlanner: Planificacion de trayectorias
- MotorController: Control de bajo nivel PID/impedancia
- MotorInterface: Interfaz de alto nivel para comandos
"""
from .motor_controller import (ControlMode, ImpedanceController,
                               ImpedanceParams, JointCommand, JointConfig,
                               JointFeedback, MotorController, PIDController,
                               PIDGains)
from .motor_interface import (CommandType, ExecutionResult, ExecutionStatus,
                              HighLevelCommand, MotorInterface)
from .trajectory_planner import (JointState, Obstacle, Pose3D, Trajectory,
                                 TrajectoryPlanner, TrajectoryType, Waypoint)

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
