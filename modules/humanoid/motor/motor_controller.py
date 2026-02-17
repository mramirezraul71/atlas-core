"""
MotorController: Control de bajo nivel de motores.

Ejecuta trayectorias y controla articulaciones con PID e impedancia.
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class ControlMode(str, Enum):
    """Modos de control."""
    POSITION = "position"       # Control de posicion
    VELOCITY = "velocity"       # Control de velocidad
    TORQUE = "torque"           # Control de torque
    IMPEDANCE = "impedance"     # Control de impedancia
    COMPLIANT = "compliant"     # Modo pasivo/compliant


@dataclass
class PIDGains:
    """Ganancias PID."""
    kp: float = 10.0   # Proporcional
    ki: float = 0.1    # Integral
    kd: float = 1.0    # Derivativo
    
    # Limites
    integral_limit: float = 10.0
    output_limit: float = 100.0


@dataclass
class ImpedanceParams:
    """Parametros de impedancia."""
    stiffness: float = 100.0   # N/m o Nm/rad
    damping: float = 10.0      # Ns/m o Nms/rad
    inertia: float = 1.0       # kg o kgm^2


@dataclass
class JointConfig:
    """Configuracion de articulacion."""
    id: str
    name: str
    
    # Limites
    position_min: float = -3.14
    position_max: float = 3.14
    velocity_max: float = 2.0
    torque_max: float = 50.0
    
    # Ganancias
    pid_gains: PIDGains = field(default_factory=PIDGains)
    impedance_params: ImpedanceParams = field(default_factory=ImpedanceParams)
    
    # Estado
    control_mode: ControlMode = ControlMode.POSITION


@dataclass
class JointCommand:
    """Comando de articulacion."""
    joint_id: str
    mode: ControlMode
    position: Optional[float] = None
    velocity: Optional[float] = None
    torque: Optional[float] = None
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())


@dataclass
class JointFeedback:
    """Feedback de articulacion."""
    joint_id: str
    position: float
    velocity: float
    torque: float
    temperature: float
    timestamp_ns: int


class PIDController:
    """Controlador PID."""
    
    def __init__(self, gains: PIDGains):
        self.gains = gains
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()
    
    def compute(self, setpoint: float, measurement: float) -> float:
        """Calcula salida PID."""
        current_time = time.time()
        dt = current_time - self._last_time
        if dt <= 0:
            dt = 0.001
        
        error = setpoint - measurement
        
        # Proporcional
        p_term = self.gains.kp * error
        
        # Integral
        self._integral += error * dt
        self._integral = max(-self.gains.integral_limit, 
                            min(self.gains.integral_limit, self._integral))
        i_term = self.gains.ki * self._integral
        
        # Derivativo
        d_term = self.gains.kd * (error - self._last_error) / dt
        
        self._last_error = error
        self._last_time = current_time
        
        # Salida
        output = p_term + i_term + d_term
        return max(-self.gains.output_limit, 
                  min(self.gains.output_limit, output))
    
    def reset(self) -> None:
        """Resetea estado del controlador."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()


class ImpedanceController:
    """Controlador de impedancia."""
    
    def __init__(self, params: ImpedanceParams):
        self.params = params
    
    def compute(
        self,
        position_desired: float,
        position_actual: float,
        velocity_actual: float,
    ) -> float:
        """
        Calcula torque de impedancia.
        
        tau = K * (x_d - x) - D * x_dot
        """
        position_error = position_desired - position_actual
        
        # Componente de stiffness (spring)
        spring_torque = self.params.stiffness * position_error
        
        # Componente de damping
        damping_torque = -self.params.damping * velocity_actual
        
        return spring_torque + damping_torque


class MotorController:
    """
    Controlador de bajo nivel de motores.
    
    Maneja:
    - Control PID por articulacion
    - Control de impedancia
    - Ejecucion de trayectorias
    - Limites de seguridad
    """
    
    def __init__(self, control_rate_hz: float = 100.0):
        """
        Inicializa el controlador.
        
        Args:
            control_rate_hz: Frecuencia de control
        """
        self.control_rate_hz = control_rate_hz
        self.control_period = 1.0 / control_rate_hz
        
        # Configuracion de joints
        self._joints: Dict[str, JointConfig] = {}
        
        # Controladores
        self._pid_controllers: Dict[str, PIDController] = {}
        self._impedance_controllers: Dict[str, ImpedanceController] = {}
        
        # Estado actual
        self._current_positions: Dict[str, float] = {}
        self._current_velocities: Dict[str, float] = {}
        self._current_torques: Dict[str, float] = {}
        
        # Comandos
        self._position_commands: Dict[str, float] = {}
        self._velocity_commands: Dict[str, float] = {}
        self._torque_commands: Dict[str, float] = {}
        
        # Estado del sistema
        self._running = False
        self._emergency_stop = False
        self._control_task: Optional[asyncio.Task] = None
        
        # Callbacks
        self._command_callbacks: List[Callable[[str, float], None]] = []
        self._feedback_callbacks: List[Callable[[JointFeedback], None]] = []
        
        # Estadisticas
        self._loop_count = 0
        self._total_control_time_ns = 0
    
    def add_joint(self, config: JointConfig) -> None:
        """Agrega configuracion de articulacion."""
        self._joints[config.id] = config
        self._pid_controllers[config.id] = PIDController(config.pid_gains)
        self._impedance_controllers[config.id] = ImpedanceController(config.impedance_params)
        self._current_positions[config.id] = 0.0
        self._current_velocities[config.id] = 0.0
        self._current_torques[config.id] = 0.0
        logger.debug(f"Added joint: {config.id}")
    
    def set_control_mode(self, joint_id: str, mode: ControlMode) -> bool:
        """Cambia modo de control de articulacion."""
        if joint_id not in self._joints:
            return False
        
        self._joints[joint_id].control_mode = mode
        
        # Resetear controladores
        if joint_id in self._pid_controllers:
            self._pid_controllers[joint_id].reset()
        
        logger.info(f"Joint {joint_id} mode changed to {mode.value}")
        return True
    
    def set_position_command(self, joint_id: str, position: float) -> bool:
        """Establece comando de posicion."""
        if joint_id not in self._joints:
            return False
        
        config = self._joints[joint_id]
        
        # Aplicar limites
        position = max(config.position_min, min(config.position_max, position))
        
        self._position_commands[joint_id] = position
        return True
    
    def set_velocity_command(self, joint_id: str, velocity: float) -> bool:
        """Establece comando de velocidad."""
        if joint_id not in self._joints:
            return False
        
        config = self._joints[joint_id]
        
        # Aplicar limites
        velocity = max(-config.velocity_max, min(config.velocity_max, velocity))
        
        self._velocity_commands[joint_id] = velocity
        return True
    
    def set_torque_command(self, joint_id: str, torque: float) -> bool:
        """Establece comando de torque."""
        if joint_id not in self._joints:
            return False
        
        config = self._joints[joint_id]
        
        # Aplicar limites
        torque = max(-config.torque_max, min(config.torque_max, torque))
        
        self._torque_commands[joint_id] = torque
        return True
    
    def update_feedback(self, feedback: JointFeedback) -> None:
        """Actualiza feedback de articulacion."""
        self._current_positions[feedback.joint_id] = feedback.position
        self._current_velocities[feedback.joint_id] = feedback.velocity
        self._current_torques[feedback.joint_id] = feedback.torque
        
        for callback in self._feedback_callbacks:
            try:
                callback(feedback)
            except Exception as e:
                logger.error(f"Error in feedback callback: {e}")
    
    async def start(self) -> None:
        """Inicia loop de control."""
        if self._running:
            return
        
        self._running = True
        self._control_task = asyncio.create_task(self._control_loop())
        logger.info("Motor controller started")
    
    async def stop(self) -> None:
        """Detiene loop de control."""
        self._running = False
        if self._control_task:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
        logger.info("Motor controller stopped")
    
    async def emergency_stop(self) -> None:
        """Parada de emergencia."""
        self._emergency_stop = True
        
        # Enviar torque cero a todos los joints
        for joint_id in self._joints:
            self._torque_commands[joint_id] = 0.0
            self._velocity_commands[joint_id] = 0.0
            
            # Notificar
            for callback in self._command_callbacks:
                try:
                    callback(joint_id, 0.0)
                except Exception as e:
                    logger.error(f"Error in emergency stop callback: {e}")
        
        logger.warning("Emergency stop activated")
    
    def reset_emergency(self) -> None:
        """Resetea estado de emergencia."""
        self._emergency_stop = False
        logger.info("Emergency stop reset")
    
    async def _control_loop(self) -> None:
        """Loop principal de control."""
        while self._running:
            start_ns = time.time_ns()
            
            if not self._emergency_stop:
                await self._compute_control()
            
            self._loop_count += 1
            self._total_control_time_ns += time.time_ns() - start_ns
            
            # Esperar hasta siguiente ciclo
            await asyncio.sleep(self.control_period)
    
    async def _compute_control(self) -> None:
        """Computa y envia comandos de control."""
        for joint_id, config in self._joints.items():
            try:
                if config.control_mode == ControlMode.POSITION:
                    output = self._compute_position_control(joint_id)
                elif config.control_mode == ControlMode.VELOCITY:
                    output = self._compute_velocity_control(joint_id)
                elif config.control_mode == ControlMode.TORQUE:
                    output = self._torque_commands.get(joint_id, 0.0)
                elif config.control_mode == ControlMode.IMPEDANCE:
                    output = self._compute_impedance_control(joint_id)
                elif config.control_mode == ControlMode.COMPLIANT:
                    output = 0.0  # Modo pasivo
                else:
                    output = 0.0
                
                # Aplicar limites de torque
                output = max(-config.torque_max, min(config.torque_max, output))
                
                # Notificar callbacks
                for callback in self._command_callbacks:
                    try:
                        callback(joint_id, output)
                    except Exception as e:
                        logger.error(f"Error in command callback: {e}")
                        
            except Exception as e:
                logger.error(f"Error computing control for {joint_id}: {e}")
    
    def _compute_position_control(self, joint_id: str) -> float:
        """Computa control de posicion con PID."""
        setpoint = self._position_commands.get(joint_id, 0.0)
        measurement = self._current_positions.get(joint_id, 0.0)
        
        pid = self._pid_controllers.get(joint_id)
        if pid:
            return pid.compute(setpoint, measurement)
        return 0.0
    
    def _compute_velocity_control(self, joint_id: str) -> float:
        """Computa control de velocidad."""
        setpoint = self._velocity_commands.get(joint_id, 0.0)
        measurement = self._current_velocities.get(joint_id, 0.0)
        
        # PID sobre velocidad
        pid = self._pid_controllers.get(joint_id)
        if pid:
            return pid.compute(setpoint, measurement)
        return 0.0
    
    def _compute_impedance_control(self, joint_id: str) -> float:
        """Computa control de impedancia."""
        position_desired = self._position_commands.get(joint_id, 0.0)
        position_actual = self._current_positions.get(joint_id, 0.0)
        velocity_actual = self._current_velocities.get(joint_id, 0.0)
        
        impedance = self._impedance_controllers.get(joint_id)
        if impedance:
            return impedance.compute(position_desired, position_actual, velocity_actual)
        return 0.0
    
    def set_pid_gains(self, joint_id: str, gains: PIDGains) -> bool:
        """Configura ganancias PID."""
        if joint_id not in self._joints:
            return False
        
        self._joints[joint_id].pid_gains = gains
        self._pid_controllers[joint_id] = PIDController(gains)
        return True
    
    def set_impedance_params(self, joint_id: str, params: ImpedanceParams) -> bool:
        """Configura parametros de impedancia."""
        if joint_id not in self._joints:
            return False
        
        self._joints[joint_id].impedance_params = params
        self._impedance_controllers[joint_id] = ImpedanceController(params)
        return True
    
    def on_command(self, callback: Callable[[str, float], None]) -> None:
        """Registra callback para comandos de motor."""
        self._command_callbacks.append(callback)
    
    def on_feedback(self, callback: Callable[[JointFeedback], None]) -> None:
        """Registra callback para feedback."""
        self._feedback_callbacks.append(callback)
    
    def get_joint_state(self, joint_id: str) -> Dict[str, float]:
        """Obtiene estado de articulacion."""
        return {
            "position": self._current_positions.get(joint_id, 0.0),
            "velocity": self._current_velocities.get(joint_id, 0.0),
            "torque": self._current_torques.get(joint_id, 0.0),
        }
    
    def get_all_positions(self) -> Dict[str, float]:
        """Obtiene todas las posiciones."""
        return dict(self._current_positions)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        avg_time = (
            self._total_control_time_ns / max(1, self._loop_count) / 1e6
        )
        return {
            "running": self._running,
            "emergency_stop": self._emergency_stop,
            "joints": len(self._joints),
            "loop_count": self._loop_count,
            "avg_control_time_ms": avg_time,
            "control_rate_hz": self.control_rate_hz,
        }
