"""
Simulation Engine: Motor de simulación física.
===============================================
Wrapper para MuJoCo/Isaac Sim con API unificada.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("simulation.engine")

SIMULATION_ROOT = Path(__file__).resolve().parent


class SimulatorBackend(str, Enum):
    """Backends de simulación soportados."""
    MUJOCO = "mujoco"
    PYBULLET = "pybullet"
    ISAAC = "isaac"
    INTERNAL = "internal"  # Simulador básico interno


@dataclass
class SimConfig:
    """Configuración del simulador."""
    backend: SimulatorBackend = SimulatorBackend.INTERNAL
    timestep: float = 0.002          # Paso de tiempo (s)
    gravity: Tuple[float, float, float] = (0, 0, -9.81)
    render: bool = False             # Renderizar visualmente
    headless: bool = True            # Sin ventana
    num_envs: int = 1                # Entornos paralelos
    max_episode_steps: int = 1000
    seed: int = 42


@dataclass
class SimState:
    """Estado de la simulación."""
    time: float
    robot_pos: np.ndarray       # Posición [x, y, z]
    robot_quat: np.ndarray      # Orientación [w, x, y, z]
    robot_vel: np.ndarray       # Velocidad lineal
    robot_ang_vel: np.ndarray   # Velocidad angular
    joint_pos: np.ndarray       # Posiciones de joints
    joint_vel: np.ndarray       # Velocidades de joints
    contacts: List[Dict[str, Any]] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "robot_pos": self.robot_pos.tolist(),
            "robot_quat": self.robot_quat.tolist(),
            "robot_vel": self.robot_vel.tolist(),
            "joint_pos": self.joint_pos.tolist(),
            "num_contacts": len(self.contacts),
        }


class SimulationEngine:
    """
    Motor de simulación para ATLAS.
    
    Proporciona una interfaz unificada para diferentes backends
    de simulación física.
    """
    
    def __init__(self, config: Optional[SimConfig] = None):
        self.config = config or SimConfig()
        self._backend = None
        self._running = False
        self._lock = threading.Lock()
        
        # Estado
        self._state = SimState(
            time=0.0,
            robot_pos=np.zeros(3),
            robot_quat=np.array([1, 0, 0, 0]),
            robot_vel=np.zeros(3),
            robot_ang_vel=np.zeros(3),
            joint_pos=np.zeros(20),
            joint_vel=np.zeros(20),
        )
        
        # Modelo del robot
        self._robot_model = None
        
        # Estadísticas
        self._stats = {
            "episodes": 0,
            "steps": 0,
            "total_reward": 0.0,
            "sim_time": 0.0,
        }
        
        _log.info("Simulation engine initialized with backend: %s", self.config.backend.value)
    
    def initialize(self) -> Dict[str, Any]:
        """Inicializa el backend de simulación."""
        try:
            if self.config.backend == SimulatorBackend.MUJOCO:
                return self._init_mujoco()
            elif self.config.backend == SimulatorBackend.PYBULLET:
                return self._init_pybullet()
            elif self.config.backend == SimulatorBackend.ISAAC:
                return self._init_isaac()
            else:
                return self._init_internal()
        except Exception as e:
            _log.exception("Failed to initialize simulator: %s", e)
            return {"ok": False, "error": str(e)}
    
    def _init_mujoco(self) -> Dict[str, Any]:
        """Inicializa MuJoCo."""
        try:
            import mujoco
            _log.info("MuJoCo version: %s", mujoco.__version__)
            return {"ok": True, "backend": "mujoco", "version": mujoco.__version__}
        except ImportError:
            _log.warning("MuJoCo not available, falling back to internal")
            self.config.backend = SimulatorBackend.INTERNAL
            return self._init_internal()
    
    def _init_pybullet(self) -> Dict[str, Any]:
        """Inicializa PyBullet."""
        try:
            import pybullet as p
            self._backend = p.connect(p.DIRECT if self.config.headless else p.GUI)
            p.setGravity(*self.config.gravity)
            p.setTimeStep(self.config.timestep)
            return {"ok": True, "backend": "pybullet"}
        except ImportError:
            _log.warning("PyBullet not available, falling back to internal")
            self.config.backend = SimulatorBackend.INTERNAL
            return self._init_internal()
    
    def _init_isaac(self) -> Dict[str, Any]:
        """Inicializa Isaac Sim (placeholder)."""
        _log.warning("Isaac Sim requires NVIDIA Omniverse, using internal")
        self.config.backend = SimulatorBackend.INTERNAL
        return self._init_internal()
    
    def _init_internal(self) -> Dict[str, Any]:
        """Inicializa simulador interno básico."""
        _log.info("Using internal physics simulation")
        return {"ok": True, "backend": "internal"}
    
    def load_robot(self, model_path: str) -> Dict[str, Any]:
        """Carga el modelo del robot."""
        from .robot_model import RobotModel
        
        try:
            self._robot_model = RobotModel()
            result = self._robot_model.load(model_path)
            if result.get("ok"):
                self._state.joint_pos = np.zeros(self._robot_model.num_joints)
                self._state.joint_vel = np.zeros(self._robot_model.num_joints)
            return result
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def reset(self, seed: Optional[int] = None) -> SimState:
        """Resetea la simulación a estado inicial."""
        if seed is not None:
            np.random.seed(seed)
        
        with self._lock:
            self._state = SimState(
                time=0.0,
                robot_pos=np.array([0.0, 0.0, 0.5]),  # Robot en origen, elevado
                robot_quat=np.array([1, 0, 0, 0]),
                robot_vel=np.zeros(3),
                robot_ang_vel=np.zeros(3),
                joint_pos=np.zeros(20),
                joint_vel=np.zeros(20),
            )
            self._stats["episodes"] += 1
        
        return self._state
    
    def step(self, action: np.ndarray) -> Tuple[SimState, float, bool, Dict[str, Any]]:
        """
        Ejecuta un paso de simulación.
        
        Args:
            action: Acciones (torques o posiciones de joints)
            
        Returns:
            (state, reward, done, info)
        """
        with self._lock:
            # Simulación física simplificada
            dt = self.config.timestep
            
            # Aplicar acción a joints
            if self._robot_model:
                # Modelo dinámico simplificado
                self._state.joint_vel += action * dt
                self._state.joint_pos += self._state.joint_vel * dt
                
                # Límites
                self._state.joint_pos = np.clip(self._state.joint_pos, -np.pi, np.pi)
            
            # Actualizar pose del robot (simplificado)
            self._state.robot_vel += np.array([0, 0, self.config.gravity[2]]) * dt
            self._state.robot_pos += self._state.robot_vel * dt
            
            # Colisión con suelo
            if self._state.robot_pos[2] < 0:
                self._state.robot_pos[2] = 0
                self._state.robot_vel[2] = 0
            
            self._state.time += dt
            self._stats["steps"] += 1
            self._stats["sim_time"] += dt
            
            # Calcular reward (placeholder)
            reward = self._compute_reward()
            self._stats["total_reward"] += reward
            
            # Verificar terminación
            done = self._check_done()
            
            info = {
                "step": self._stats["steps"],
                "sim_time": self._state.time,
            }
            
            return self._state, reward, done, info
    
    def _compute_reward(self) -> float:
        """Calcula el reward (placeholder)."""
        # Reward por mantenerse de pie
        height_reward = max(0, self._state.robot_pos[2] - 0.3)
        # Penalización por velocidad alta
        vel_penalty = -0.01 * np.linalg.norm(self._state.robot_vel)
        return height_reward + vel_penalty
    
    def _check_done(self) -> bool:
        """Verifica si el episodio terminó."""
        # Terminado si el robot cae
        if self._state.robot_pos[2] < 0.1:
            return True
        # Terminado si excede pasos máximos
        if self._stats["steps"] % self.config.max_episode_steps == 0:
            return True
        return False
    
    def run_episode(self, policy=None, max_steps: Optional[int] = None) -> Dict[str, Any]:
        """
        Ejecuta un episodio completo.
        
        Args:
            policy: Función que recibe state y retorna action
            max_steps: Pasos máximos (override config)
            
        Returns:
            Resultados del episodio
        """
        state = self.reset()
        total_reward = 0.0
        steps = 0
        max_steps = max_steps or self.config.max_episode_steps
        
        while steps < max_steps:
            # Obtener acción
            if policy:
                action = policy(state)
            else:
                action = np.zeros(20)  # Acción nula
            
            # Ejecutar paso
            state, reward, done, info = self.step(action)
            total_reward += reward
            steps += 1
            
            if done:
                break
        
        return {
            "total_reward": total_reward,
            "steps": steps,
            "final_pos": state.robot_pos.tolist(),
            "success": state.robot_pos[2] > 0.3,
        }
    
    def get_state(self) -> SimState:
        """Retorna el estado actual."""
        with self._lock:
            return self._state
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas de simulación."""
        return {
            **self._stats,
            "backend": self.config.backend.value,
            "num_envs": self.config.num_envs,
        }
    
    def close(self) -> None:
        """Cierra el simulador."""
        if self.config.backend == SimulatorBackend.PYBULLET and self._backend:
            try:
                import pybullet as p
                p.disconnect(self._backend)
            except Exception:
                pass
        _log.info("Simulation engine closed")
