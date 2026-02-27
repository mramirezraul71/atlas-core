"""
Training Environment: Entornos para entrenamiento RL.
======================================================
Define entornos tipo Gymnasium para entrenar políticas.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("simulation.training_env")


@dataclass
class EnvConfig:
    """Configuración del entorno de entrenamiento."""
    task: str = "locomotion"  # locomotion, manipulation, navigation
    max_episode_steps: int = 1000
    reward_scale: float = 1.0
    observation_noise: float = 0.01
    action_noise: float = 0.0
    early_termination: bool = True


class TrainingEnvironment:
    """
    Entorno de entrenamiento compatible con Gymnasium.
    
    Tareas disponibles:
    - locomotion: Caminar/correr
    - stand: Mantenerse de pie
    - manipulation: Manipular objetos
    - navigation: Navegar a puntos
    """
    
    def __init__(self, config: Optional[EnvConfig] = None):
        self.config = config or EnvConfig()
        
        # Espacios
        self._observation_dim = 50  # Posiciones, velocidades, etc.
        self._action_dim = 20       # Torques de joints
        
        # Estado
        self._state: Optional[np.ndarray] = None
        self._step_count: int = 0
        self._episode_reward: float = 0.0
        
        # Objetivo (para tareas de navegación)
        self._target_pos: np.ndarray = np.zeros(3)
        
        _log.info("Training environment created for task: %s", self.config.task)
    
    @property
    def observation_space(self) -> Dict[str, Any]:
        """Retorna descripción del espacio de observación."""
        return {
            "shape": (self._observation_dim,),
            "dtype": "float32",
            "low": -np.inf,
            "high": np.inf,
        }
    
    @property
    def action_space(self) -> Dict[str, Any]:
        """Retorna descripción del espacio de acción."""
        return {
            "shape": (self._action_dim,),
            "dtype": "float32",
            "low": -1.0,
            "high": 1.0,
        }
    
    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Resetea el entorno.
        
        Returns:
            (observation, info)
        """
        if seed is not None:
            np.random.seed(seed)
        
        self._step_count = 0
        self._episode_reward = 0.0
        
        # Estado inicial con algo de ruido
        self._state = np.zeros(self._observation_dim)
        self._state[:3] = [0, 0, 0.5]  # Posición inicial
        self._state += np.random.normal(0, 0.01, self._observation_dim)
        
        # Nuevo objetivo para tareas de navegación
        if self.config.task == "navigation":
            self._target_pos = np.random.uniform(-2, 2, 3)
            self._target_pos[2] = 0.5
        
        info = {"target": self._target_pos.tolist()}
        
        return self._state.copy(), info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Ejecuta un paso del entorno.
        
        Args:
            action: Acción a ejecutar
            
        Returns:
            (observation, reward, terminated, truncated, info)
        """
        # Aplicar ruido a acción
        if self.config.action_noise > 0:
            action = action + np.random.normal(0, self.config.action_noise, action.shape)
        
        # Clip acción
        action = np.clip(action, -1, 1)
        
        # Simular dinámica (simplificada)
        self._state = self._dynamics_step(self._state, action)
        
        # Aplicar ruido a observación
        obs = self._state + np.random.normal(0, self.config.observation_noise, self._state.shape)
        
        # Calcular reward
        reward = self._compute_reward(self._state, action)
        self._episode_reward += reward
        
        # Verificar terminación
        terminated = self._check_terminated()
        truncated = self._step_count >= self.config.max_episode_steps
        
        self._step_count += 1
        
        info = {
            "step": self._step_count,
            "episode_reward": self._episode_reward,
        }
        
        return obs.astype(np.float32), float(reward), terminated, truncated, info
    
    def _dynamics_step(self, state: np.ndarray, action: np.ndarray) -> np.ndarray:
        """Paso de dinámica simplificada."""
        new_state = state.copy()
        
        dt = 0.01
        
        # Posición y velocidad del cuerpo
        pos = new_state[:3]
        vel = new_state[3:6]
        
        # Aplicar gravedad
        vel[2] -= 9.81 * dt
        
        # Aplicar acción (simplificado: acción afecta velocidad)
        vel[:2] += action[:2] * 0.1 * dt
        
        # Actualizar posición
        pos += vel * dt
        
        # Colisión con suelo
        if pos[2] < 0:
            pos[2] = 0
            vel[2] = 0
        
        new_state[:3] = pos
        new_state[3:6] = vel
        
        return new_state
    
    def _compute_reward(self, state: np.ndarray, action: np.ndarray) -> float:
        """Calcula el reward según la tarea."""
        if self.config.task == "locomotion":
            # Reward por velocidad hacia adelante
            forward_vel = state[3]  # velocidad x
            height = state[2]
            
            reward = forward_vel * 0.5  # Recompensa velocidad
            reward += 0.2 * (height - 0.3)  # Bonus por altura
            reward -= 0.01 * np.sum(action**2)  # Penalización esfuerzo
            
        elif self.config.task == "stand":
            # Reward por mantenerse de pie
            height = state[2]
            vel_magnitude = np.linalg.norm(state[3:6])
            
            reward = 1.0 if height > 0.4 else 0.0
            reward -= 0.1 * vel_magnitude  # Penalizar movimiento
            
        elif self.config.task == "navigation":
            # Reward por acercarse al objetivo
            pos = state[:3]
            dist = np.linalg.norm(pos - self._target_pos)
            
            reward = -dist * 0.1  # Penalización por distancia
            if dist < 0.2:
                reward += 10.0  # Bonus por llegar
        
        else:
            reward = 0.0
        
        return reward * self.config.reward_scale
    
    def _check_terminated(self) -> bool:
        """Verifica si el episodio debe terminar."""
        if not self.config.early_termination:
            return False
        
        height = self._state[2]
        
        # Terminar si el robot cae
        if height < 0.1:
            return True
        
        # Terminar si navegación completada
        if self.config.task == "navigation":
            dist = np.linalg.norm(self._state[:3] - self._target_pos)
            if dist < 0.2:
                return True
        
        return False
    
    def render(self, mode: str = "human") -> Optional[np.ndarray]:
        """Renderiza el entorno (placeholder)."""
        # TODO: Implementar renderizado real
        return None
    
    def close(self) -> None:
        """Cierra el entorno."""
        pass
    
    def get_info(self) -> Dict[str, Any]:
        """Retorna información del entorno."""
        return {
            "task": self.config.task,
            "observation_dim": self._observation_dim,
            "action_dim": self._action_dim,
            "max_episode_steps": self.config.max_episode_steps,
            "current_step": self._step_count,
            "episode_reward": self._episode_reward,
        }
