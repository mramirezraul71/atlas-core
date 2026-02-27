"""
MotorInterface: Interfaz de alto nivel entre planificador cognitivo y control motor.

Traduce comandos de alto nivel a secuencias motoras ejecutables.
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import Enum

from .trajectory_planner import TrajectoryPlanner, Trajectory, Pose3D, TrajectoryType
from .motor_controller import MotorController, ControlMode, JointConfig

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="motor")
    except Exception:
        pass


class CommandType(str, Enum):
    """Tipos de comandos de alto nivel."""
    MOVE_TO = "move_to"         # Navegar a ubicacion
    GRASP = "grasp"             # Agarrar objeto
    PLACE = "place"             # Colocar objeto
    RELEASE = "release"         # Soltar
    POINT_AT = "point_at"       # Señalar
    WAVE = "wave"               # Saludar
    NOD = "nod"                 # Asentir
    SHAKE_HEAD = "shake_head"   # Negar
    LOOK_AT = "look_at"         # Mirar hacia
    HOME = "home"               # Posicion inicial


class ExecutionStatus(str, Enum):
    """Estado de ejecucion."""
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class HighLevelCommand:
    """Comando de alto nivel."""
    id: str
    command_type: CommandType
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    # Configuracion
    speed: float = 1.0          # Factor de velocidad (0.1 - 2.0)
    precision: float = 0.01     # Precision requerida (m)
    timeout: float = 30.0       # Timeout en segundos
    
    # Estado
    status: ExecutionStatus = ExecutionStatus.PENDING
    start_time_ns: Optional[int] = None
    end_time_ns: Optional[int] = None
    error: Optional[str] = None
    
    # Metadata
    created_at_ns: int = field(default_factory=lambda: time.time_ns())


@dataclass
class ExecutionResult:
    """Resultado de ejecucion."""
    command_id: str
    success: bool
    duration_ms: float
    error: Optional[str] = None
    details: Dict[str, Any] = field(default_factory=dict)


class MotorInterface:
    """
    Interfaz de alto nivel para control motor.
    
    Traduce comandos cognitivos a movimientos:
    - MoveTo: navegacion
    - Grasp: secuencia de agarre
    - Place: secuencia de colocacion
    - Gestos: wave, nod, point, etc.
    """
    
    def __init__(
        self,
        trajectory_planner: Optional[TrajectoryPlanner] = None,
        motor_controller: Optional[MotorController] = None,
    ):
        """
        Inicializa la interfaz.
        
        Args:
            trajectory_planner: Planificador de trayectorias
            motor_controller: Controlador de motores
        """
        self.trajectory_planner = trajectory_planner or TrajectoryPlanner()
        self.motor_controller = motor_controller or MotorController()
        
        # Cola de comandos
        self._command_queue: List[HighLevelCommand] = []
        self._current_command: Optional[HighLevelCommand] = None
        
        # Historial
        self._execution_history: List[ExecutionResult] = []
        
        # Estado
        self._executing = False
        self._executor_task: Optional[asyncio.Task] = None
        
        # Posiciones predefinidas
        self._home_positions: Dict[str, float] = {}
        self._saved_poses: Dict[str, Dict[str, float]] = {}
        
        # Gripper
        self._gripper_closed = False
        
        # Callbacks
        self._on_start_callbacks: List[Callable[[HighLevelCommand], None]] = []
        self._on_complete_callbacks: List[Callable[[ExecutionResult], None]] = []
        
        # Contador
        self._command_count = 0
        
        # Configurar posiciones default
        self._init_default_poses()
    
    def _init_default_poses(self) -> None:
        """Inicializa poses predefinidas."""
        # Posicion home
        self._home_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -1.57,
            "elbow": 1.57,
            "wrist_1": 0.0,
            "wrist_2": -1.57,
            "wrist_3": 0.0,
        }
        
        # Poses guardadas
        self._saved_poses["wave_up"] = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.5,
            "elbow": 1.0,
            "wrist_1": 0.5,
        }
        
        self._saved_poses["wave_down"] = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.5,
            "elbow": 1.0,
            "wrist_1": -0.5,
        }
        
        self._saved_poses["point_forward"] = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -0.8,
            "elbow": 0.0,
            "wrist_1": 0.0,
        }
    
    async def execute(self, command: HighLevelCommand) -> ExecutionResult:
        """
        Ejecuta comando de alto nivel.
        
        Args:
            command: Comando a ejecutar
        
        Returns:
            Resultado de la ejecucion
        """
        command.status = ExecutionStatus.EXECUTING
        command.start_time_ns = time.time_ns()
        self._current_command = command
        
        # Bitácora
        target = command.parameters.get("target") or command.parameters.get("pose") or command.parameters.get("object_pose")
        _bitacora(f"Motor cmd: {command.command_type} target={target}")
        
        # Notificar inicio
        for callback in self._on_start_callbacks:
            try:
                callback(command)
            except Exception as e:
                logger.error(f"Error in on_start callback: {e}")
        
        try:
            # Ejecutar segun tipo
            if command.command_type == CommandType.MOVE_TO:
                result = await self._execute_move_to(command)
            elif command.command_type == CommandType.GRASP:
                result = await self._execute_grasp(command)
            elif command.command_type == CommandType.PLACE:
                result = await self._execute_place(command)
            elif command.command_type == CommandType.RELEASE:
                result = await self._execute_release(command)
            elif command.command_type == CommandType.POINT_AT:
                result = await self._execute_point_at(command)
            elif command.command_type == CommandType.WAVE:
                result = await self._execute_wave(command)
            elif command.command_type == CommandType.NOD:
                result = await self._execute_nod(command)
            elif command.command_type == CommandType.SHAKE_HEAD:
                result = await self._execute_shake_head(command)
            elif command.command_type == CommandType.LOOK_AT:
                result = await self._execute_look_at(command)
            elif command.command_type == CommandType.HOME:
                result = await self._execute_home(command)
            else:
                result = ExecutionResult(
                    command_id=command.id,
                    success=False,
                    duration_ms=0,
                    error=f"Unknown command type: {command.command_type}",
                )
            
        except asyncio.CancelledError:
            command.status = ExecutionStatus.CANCELLED
            result = ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
                error="Execution cancelled",
            )
        except Exception as e:
            logger.error(f"Error executing command {command.id}: {e}")
            command.status = ExecutionStatus.FAILED
            command.error = str(e)
            result = ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
                error=str(e),
            )
        
        command.end_time_ns = time.time_ns()
        self._current_command = None
        
        # Bitácora de resultado
        if result.success:
            _bitacora(f"Motor cmd completado: {command.id} dur={result.duration_ms:.0f}ms")
        else:
            _bitacora(f"Motor cmd falló: {command.id} error={result.error}", ok=False)
        
        # Guardar en historial
        self._execution_history.append(result)
        if len(self._execution_history) > 1000:
            self._execution_history = self._execution_history[-500:]
        
        # Notificar completado
        for callback in self._on_complete_callbacks:
            try:
                callback(result)
            except Exception as e:
                logger.error(f"Error in on_complete callback: {e}")
        
        return result
    
    async def _execute_move_to(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta movimiento a pose objetivo."""
        target_pose = command.parameters.get("pose")
        target_joints = command.parameters.get("joints")
        
        if target_pose:
            # Movimiento cartesiano
            pose = Pose3D(**target_pose) if isinstance(target_pose, dict) else target_pose
            
            # Obtener pose actual (mock)
            current_pose = Pose3D(0, 0, 0, 0, 0, 0)
            
            # Planificar
            trajectory = self.trajectory_planner.plan_cartesian_trajectory(
                current_pose=current_pose,
                target_pose=pose,
                trajectory_type=TrajectoryType.MINIMUM_JERK,
            )
        elif target_joints:
            # Movimiento en espacio de joints
            current_joints = self.motor_controller.get_all_positions()
            
            trajectory = self.trajectory_planner.plan_joint_trajectory(
                current_joints=current_joints,
                target_joints=target_joints,
                trajectory_type=TrajectoryType.QUINTIC,
            )
        else:
            return ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=0,
                error="No target pose or joints specified",
            )
        
        # Validar trayectoria
        if not trajectory.is_valid:
            return ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=0,
                error=f"Invalid trajectory: {trajectory.validation_errors}",
            )
        
        # Ejecutar trayectoria
        success = await self._execute_trajectory(trajectory, command.speed)
        
        command.status = ExecutionStatus.COMPLETED if success else ExecutionStatus.FAILED
        
        return ExecutionResult(
            command_id=command.id,
            success=success,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
            details={"trajectory_id": trajectory.id, "duration": trajectory.duration},
        )
    
    async def _execute_grasp(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta secuencia de agarre."""
        object_pose = command.parameters.get("object_pose")
        approach_distance = command.parameters.get("approach_distance", 0.1)
        
        if not object_pose:
            return ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=0,
                error="No object pose specified",
            )
        
        pose = Pose3D(**object_pose) if isinstance(object_pose, dict) else object_pose
        
        # 1. Aproximar
        approach_pose = Pose3D(
            x=pose.x,
            y=pose.y,
            z=pose.z + approach_distance,
            roll=pose.roll,
            pitch=pose.pitch,
            yaw=pose.yaw,
        )
        
        current_pose = Pose3D(0, 0, 0, 0, 0, 0)  # Mock
        
        approach_traj = self.trajectory_planner.plan_cartesian_trajectory(
            current_pose, approach_pose
        )
        await self._execute_trajectory(approach_traj, command.speed)
        
        # 2. Descender al objeto
        grasp_traj = self.trajectory_planner.plan_cartesian_trajectory(
            approach_pose, pose, trajectory_type=TrajectoryType.LINEAR
        )
        await self._execute_trajectory(grasp_traj, command.speed * 0.5)
        
        # 3. Cerrar gripper
        await self._close_gripper()
        
        # 4. Retirarse
        retreat_traj = self.trajectory_planner.plan_cartesian_trajectory(
            pose, approach_pose
        )
        await self._execute_trajectory(retreat_traj, command.speed * 0.5)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
            details={"gripper_closed": self._gripper_closed},
        )
    
    async def _execute_place(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta secuencia de colocacion."""
        target_pose = command.parameters.get("target_pose")
        approach_distance = command.parameters.get("approach_distance", 0.1)
        
        if not target_pose:
            return ExecutionResult(
                command_id=command.id,
                success=False,
                duration_ms=0,
                error="No target pose specified",
            )
        
        pose = Pose3D(**target_pose) if isinstance(target_pose, dict) else target_pose
        
        # 1. Aproximar
        approach_pose = Pose3D(
            x=pose.x,
            y=pose.y,
            z=pose.z + approach_distance,
            roll=pose.roll,
            pitch=pose.pitch,
            yaw=pose.yaw,
        )
        
        current_pose = Pose3D(0, 0, 0.2, 0, 0, 0)  # Mock
        
        approach_traj = self.trajectory_planner.plan_cartesian_trajectory(
            current_pose, approach_pose
        )
        await self._execute_trajectory(approach_traj, command.speed)
        
        # 2. Descender
        place_traj = self.trajectory_planner.plan_cartesian_trajectory(
            approach_pose, pose, trajectory_type=TrajectoryType.LINEAR
        )
        await self._execute_trajectory(place_traj, command.speed * 0.5)
        
        # 3. Abrir gripper
        await self._open_gripper()
        
        # 4. Retirarse
        retreat_traj = self.trajectory_planner.plan_cartesian_trajectory(
            pose, approach_pose
        )
        await self._execute_trajectory(retreat_traj, command.speed * 0.5)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_release(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta soltar objeto."""
        await self._open_gripper()
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_point_at(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta señalar."""
        target = command.parameters.get("target")
        
        # Mover a pose de señalar
        point_joints = self._saved_poses.get("point_forward", {})
        
        current_joints = self.motor_controller.get_all_positions()
        
        trajectory = self.trajectory_planner.plan_joint_trajectory(
            current_joints, point_joints
        )
        await self._execute_trajectory(trajectory, command.speed)
        
        # Mantener pose un momento
        await asyncio.sleep(1.0)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_wave(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta gesto de saludo."""
        cycles = command.parameters.get("cycles", 3)
        
        current_joints = self.motor_controller.get_all_positions()
        wave_up = self._saved_poses.get("wave_up", {})
        wave_down = self._saved_poses.get("wave_down", {})
        
        # Ir a posicion inicial
        init_traj = self.trajectory_planner.plan_joint_trajectory(
            current_joints, wave_up
        )
        await self._execute_trajectory(init_traj, command.speed)
        
        # Ciclos de saludo
        for _ in range(cycles):
            down_traj = self.trajectory_planner.plan_joint_trajectory(
                wave_up, wave_down, duration=0.3
            )
            await self._execute_trajectory(down_traj, command.speed * 1.5)
            
            up_traj = self.trajectory_planner.plan_joint_trajectory(
                wave_down, wave_up, duration=0.3
            )
            await self._execute_trajectory(up_traj, command.speed * 1.5)
        
        # Volver a home
        await self._execute_home(command)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
            details={"cycles": cycles},
        )
    
    async def _execute_nod(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta gesto de asentir."""
        # Simular movimiento de cabeza (si hay neck joints)
        await asyncio.sleep(0.5)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_shake_head(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta gesto de negar."""
        # Simular movimiento de cabeza
        await asyncio.sleep(0.5)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_look_at(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta mirar hacia objetivo."""
        target = command.parameters.get("target")
        
        # Calcular orientacion de cabeza/ojos
        await asyncio.sleep(0.3)
        
        command.status = ExecutionStatus.COMPLETED
        
        return ExecutionResult(
            command_id=command.id,
            success=True,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_home(self, command: HighLevelCommand) -> ExecutionResult:
        """Ejecuta ir a posicion home."""
        current_joints = self.motor_controller.get_all_positions()
        
        trajectory = self.trajectory_planner.plan_joint_trajectory(
            current_joints, self._home_positions,
            trajectory_type=TrajectoryType.MINIMUM_JERK,
        )
        
        success = await self._execute_trajectory(trajectory, command.speed)
        
        command.status = ExecutionStatus.COMPLETED if success else ExecutionStatus.FAILED
        
        return ExecutionResult(
            command_id=command.id,
            success=success,
            duration_ms=(time.time_ns() - command.start_time_ns) / 1e6,
        )
    
    async def _execute_trajectory(
        self,
        trajectory: Trajectory,
        speed_factor: float = 1.0,
    ) -> bool:
        """Ejecuta una trayectoria."""
        if not trajectory.waypoints:
            return False
        
        duration = trajectory.get_duration() / speed_factor
        start_time = time.time()
        
        while True:
            elapsed = time.time() - start_time
            
            if elapsed >= duration:
                # Enviar ultimo waypoint
                final = trajectory.waypoints[-1]
                if final.joints:
                    for joint_id, pos in final.joints.items():
                        self.motor_controller.set_position_command(joint_id, pos)
                break
            
            # Samplear trayectoria
            t = elapsed * speed_factor
            waypoint = trajectory.sample(t)
            
            if waypoint and waypoint.joints:
                for joint_id, pos in waypoint.joints.items():
                    self.motor_controller.set_position_command(joint_id, pos)
            
            await asyncio.sleep(0.01)  # 100 Hz
        
        return True
    
    async def _close_gripper(self) -> bool:
        """Cierra gripper."""
        logger.debug("Closing gripper")
        self._gripper_closed = True
        await asyncio.sleep(0.5)  # Simular tiempo de cierre
        return True
    
    async def _open_gripper(self) -> bool:
        """Abre gripper."""
        logger.debug("Opening gripper")
        self._gripper_closed = False
        await asyncio.sleep(0.3)  # Simular tiempo de apertura
        return True
    
    def create_command(
        self,
        command_type: CommandType,
        parameters: Dict[str, Any] = None,
        **kwargs,
    ) -> HighLevelCommand:
        """Crea comando de alto nivel."""
        self._command_count += 1
        return HighLevelCommand(
            id=f"cmd_{self._command_count}",
            command_type=command_type,
            parameters=parameters or {},
            **kwargs,
        )
    
    async def move_to(self, pose: Pose3D, speed: float = 1.0) -> ExecutionResult:
        """Shortcut para mover a pose."""
        cmd = self.create_command(
            CommandType.MOVE_TO,
            parameters={"pose": pose},
            speed=speed,
        )
        return await self.execute(cmd)
    
    async def grasp(self, object_pose: Pose3D) -> ExecutionResult:
        """Shortcut para agarrar objeto."""
        cmd = self.create_command(
            CommandType.GRASP,
            parameters={"object_pose": object_pose},
        )
        return await self.execute(cmd)
    
    async def place(self, target_pose: Pose3D) -> ExecutionResult:
        """Shortcut para colocar objeto."""
        cmd = self.create_command(
            CommandType.PLACE,
            parameters={"target_pose": target_pose},
        )
        return await self.execute(cmd)
    
    async def wave(self, cycles: int = 3) -> ExecutionResult:
        """Shortcut para saludar."""
        cmd = self.create_command(
            CommandType.WAVE,
            parameters={"cycles": cycles},
        )
        return await self.execute(cmd)
    
    async def go_home(self) -> ExecutionResult:
        """Shortcut para ir a home."""
        cmd = self.create_command(CommandType.HOME)
        return await self.execute(cmd)
    
    def on_start(self, callback: Callable[[HighLevelCommand], None]) -> None:
        """Registra callback para inicio de comando."""
        self._on_start_callbacks.append(callback)
    
    def on_complete(self, callback: Callable[[ExecutionResult], None]) -> None:
        """Registra callback para completado de comando."""
        self._on_complete_callbacks.append(callback)
    
    def set_home_position(self, positions: Dict[str, float]) -> None:
        """Configura posicion home."""
        self._home_positions = positions
    
    def save_pose(self, name: str, positions: Dict[str, float]) -> None:
        """Guarda pose con nombre."""
        self._saved_poses[name] = positions
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        success_count = sum(1 for r in self._execution_history if r.success)
        return {
            "total_commands": self._command_count,
            "success_count": success_count,
            "failure_count": self._command_count - success_count,
            "success_rate": success_count / max(1, self._command_count),
            "gripper_closed": self._gripper_closed,
            "saved_poses": list(self._saved_poses.keys()),
        }
