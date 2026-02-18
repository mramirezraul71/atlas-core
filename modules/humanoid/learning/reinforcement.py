"""
ReinforcementLearning: Aprendizaje por refuerzo.

Sistema de RL para aprender y mejorar comportamientos.
"""
from __future__ import annotations

import asyncio
import json
import logging
import random
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="learning")
    except Exception:
        pass


@dataclass
class Experience:
    """Experiencia individual (s, a, r, s', done)."""
    state: Dict[str, float]
    action: Dict[str, float]
    reward: float
    next_state: Dict[str, float]
    done: bool
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())


@dataclass
class EpisodeResult:
    """Resultado de un episodio de entrenamiento."""
    episode_id: int
    total_reward: float
    steps: int
    duration_ms: float
    success: bool


class ReplayBuffer:
    """Buffer de experiencias para experience replay."""
    
    def __init__(self, capacity: int = 10000):
        self.buffer: deque = deque(maxlen=capacity)
    
    def add(self, experience: Experience) -> None:
        """Agrega experiencia al buffer."""
        self.buffer.append(experience)
    
    def sample(self, batch_size: int) -> List[Experience]:
        """Muestrea batch aleatorio."""
        if len(self.buffer) < batch_size:
            return list(self.buffer)
        return random.sample(list(self.buffer), batch_size)
    
    def __len__(self) -> int:
        return len(self.buffer)
    
    def clear(self) -> None:
        self.buffer.clear()


class Policy:
    """
    Politica simple basada en tabla Q.
    
    Para estados continuos, discretiza el espacio de estados.
    """
    
    def __init__(self, 
                 action_space: List[str],
                 learning_rate: float = 0.1,
                 discount_factor: float = 0.99,
                 epsilon: float = 0.1):
        """
        Inicializa la politica.
        
        Args:
            action_space: Lista de nombres de acciones
            learning_rate: Tasa de aprendizaje
            discount_factor: Factor de descuento
            epsilon: Probabilidad de exploracion
        """
        self.action_space = action_space
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        
        # Tabla Q simplificada
        self._q_table: Dict[str, Dict[str, float]] = {}
        
        # Estadisticas
        self._updates = 0
    
    def _discretize_state(self, state: Dict[str, float]) -> str:
        """Discretiza estado a string para usar como clave."""
        # Discretizacion simple: redondear valores
        parts = []
        for key in sorted(state.keys()):
            value = state[key]
            discretized = round(value, 1)
            parts.append(f"{key}:{discretized}")
        return "|".join(parts)
    
    def get_q_values(self, state: Dict[str, float]) -> Dict[str, float]:
        """Obtiene valores Q para un estado."""
        state_key = self._discretize_state(state)
        
        if state_key not in self._q_table:
            # Inicializar con valores pequenos aleatorios
            self._q_table[state_key] = {
                action: random.uniform(-0.1, 0.1)
                for action in self.action_space
            }
        
        return self._q_table[state_key]
    
    def select_action(self, state: Dict[str, float], 
                     explore: bool = True) -> str:
        """
        Selecciona accion usando epsilon-greedy.
        
        Args:
            state: Estado actual
            explore: Si permitir exploracion
        
        Returns:
            Nombre de la accion seleccionada
        """
        if explore and random.random() < self.epsilon:
            # Exploracion: accion aleatoria
            return random.choice(self.action_space)
        
        # Explotacion: mejor accion
        q_values = self.get_q_values(state)
        return max(q_values.keys(), key=lambda a: q_values[a])
    
    def update(self, state: Dict[str, float], action: str,
              reward: float, next_state: Dict[str, float],
              done: bool) -> float:
        """
        Actualiza valor Q usando Q-learning.
        
        Returns:
            TD error
        """
        state_key = self._discretize_state(state)
        next_state_key = self._discretize_state(next_state)
        
        # Asegurar que existen entradas
        if state_key not in self._q_table:
            self.get_q_values(state)
        
        current_q = self._q_table[state_key][action]
        
        if done:
            target_q = reward
        else:
            next_q_values = self.get_q_values(next_state)
            max_next_q = max(next_q_values.values())
            target_q = reward + self.discount_factor * max_next_q
        
        # TD error
        td_error = target_q - current_q
        
        # Actualizar
        self._q_table[state_key][action] += self.learning_rate * td_error
        self._updates += 1
        
        return td_error
    
    def decay_epsilon(self, min_epsilon: float = 0.01, 
                     decay_rate: float = 0.995) -> None:
        """Reduce epsilon para menos exploracion."""
        self.epsilon = max(min_epsilon, self.epsilon * decay_rate)
    
    def save(self, filepath: str) -> None:
        """Guarda politica a archivo."""
        data = {
            "action_space": self.action_space,
            "learning_rate": self.learning_rate,
            "discount_factor": self.discount_factor,
            "epsilon": self.epsilon,
            "q_table": self._q_table,
            "updates": self._updates,
        }
        with open(filepath, "w") as f:
            json.dump(data, f)
    
    @classmethod
    def load(cls, filepath: str) -> "Policy":
        """Carga politica desde archivo."""
        with open(filepath, "r") as f:
            data = json.load(f)
        
        policy = cls(
            action_space=data["action_space"],
            learning_rate=data.get("learning_rate", 0.1),
            discount_factor=data.get("discount_factor", 0.99),
            epsilon=data.get("epsilon", 0.1),
        )
        policy._q_table = data.get("q_table", {})
        policy._updates = data.get("updates", 0)
        
        return policy
    
    def get_stats(self) -> Dict[str, Any]:
        return {
            "action_space_size": len(self.action_space),
            "epsilon": self.epsilon,
            "q_table_size": len(self._q_table),
            "total_updates": self._updates,
        }


class ReinforcementLearning:
    """
    Sistema de aprendizaje por refuerzo.
    
    Permite:
    - Entrenar politicas por episodios
    - Usar reward shaping
    - Experience replay
    """
    
    def __init__(self,
                 storage_path: str = None,
                 batch_size: int = 32,
                 buffer_capacity: int = 10000):
        """
        Inicializa el sistema de RL.
        
        Args:
            storage_path: Ruta para guardar politicas
            batch_size: Tamano de batch para replay
            buffer_capacity: Capacidad del replay buffer
        """
        self.storage_path = Path(storage_path) if storage_path else None
        self.batch_size = batch_size
        
        # Politicas por tarea
        self._policies: Dict[str, Policy] = {}
        
        # Replay buffer por tarea
        self._buffers: Dict[str, ReplayBuffer] = {}
        self.buffer_capacity = buffer_capacity
        
        # Historial de entrenamiento
        self._episode_history: Dict[str, List[EpisodeResult]] = {}
        
        # Funciones de recompensa personalizadas
        self._reward_functions: Dict[str, Callable] = {}
        
        # Estado de entrenamiento
        self._training = False
        self._current_task: Optional[str] = None
        self._episode_count: Dict[str, int] = {}
    
    def create_policy(self, task_id: str,
                     action_space: List[str],
                     **kwargs) -> Policy:
        """
        Crea politica para una tarea.
        
        Args:
            task_id: ID de la tarea
            action_space: Espacio de acciones
            **kwargs: Parametros adicionales para Policy
        
        Returns:
            Policy creada
        """
        policy = Policy(action_space=action_space, **kwargs)
        self._policies[task_id] = policy
        self._buffers[task_id] = ReplayBuffer(self.buffer_capacity)
        self._episode_history[task_id] = []
        self._episode_count[task_id] = 0
        
        logger.info(f"Created policy for task {task_id} with {len(action_space)} actions")
        return policy
    
    def get_policy(self, task_id: str) -> Optional[Policy]:
        """Obtiene politica para una tarea."""
        return self._policies.get(task_id)
    
    def set_reward_function(self, task_id: str,
                           reward_fn: Callable[[Dict, str, Dict], float]) -> None:
        """
        Establece funcion de recompensa personalizada.
        
        Args:
            task_id: ID de la tarea
            reward_fn: Funcion (state, action, next_state) -> reward
        """
        self._reward_functions[task_id] = reward_fn
    
    def compute_reward(self, task_id: str,
                      state: Dict[str, float],
                      action: str,
                      next_state: Dict[str, float],
                      base_reward: float = 0.0) -> float:
        """
        Calcula recompensa, usando funcion personalizada si existe.
        
        Args:
            task_id: ID de la tarea
            state: Estado anterior
            action: Accion tomada
            next_state: Estado resultante
            base_reward: Recompensa base del ambiente
        
        Returns:
            Recompensa total
        """
        if task_id in self._reward_functions:
            try:
                custom_reward = self._reward_functions[task_id](state, action, next_state)
                return base_reward + custom_reward
            except Exception as e:
                logger.error(f"Error in reward function: {e}")
        
        return base_reward
    
    def record_experience(self, task_id: str,
                         state: Dict[str, float],
                         action: str,
                         reward: float,
                         next_state: Dict[str, float],
                         done: bool) -> None:
        """
        Registra experiencia y actualiza politica.
        
        Args:
            task_id: ID de la tarea
            state: Estado anterior
            action: Accion tomada
            reward: Recompensa recibida
            next_state: Estado resultante
            done: Si el episodio termino
        """
        if task_id not in self._policies:
            logger.warning(f"No policy for task {task_id}")
            return
        
        # Agregar a buffer
        experience = Experience(
            state=state,
            action={"action": action} if isinstance(action, str) else action,
            reward=reward,
            next_state=next_state,
            done=done,
        )
        self._buffers[task_id].add(experience)
        
        # Actualizar politica directamente
        policy = self._policies[task_id]
        policy.update(state, action, reward, next_state, done)
    
    async def train_batch(self, task_id: str) -> float:
        """
        Entrena con un batch del replay buffer.
        
        Args:
            task_id: ID de la tarea
        
        Returns:
            TD error promedio
        """
        if task_id not in self._policies:
            return 0.0
        
        buffer = self._buffers.get(task_id)
        if not buffer or len(buffer) < self.batch_size:
            return 0.0
        
        policy = self._policies[task_id]
        batch = buffer.sample(self.batch_size)
        
        total_error = 0.0
        for exp in batch:
            action = exp.action.get("action") if isinstance(exp.action, dict) else exp.action
            error = policy.update(exp.state, action, exp.reward, exp.next_state, exp.done)
            total_error += abs(error)
        
        return total_error / len(batch)
    
    def end_episode(self, task_id: str,
                   total_reward: float,
                   steps: int,
                   success: bool,
                   duration_ms: float = 0.0) -> EpisodeResult:
        """
        Registra fin de episodio.
        
        Args:
            task_id: ID de la tarea
            total_reward: Recompensa total
            steps: Numero de pasos
            success: Si fue exitoso
            duration_ms: Duracion
        
        Returns:
            Resultado del episodio
        """
        self._episode_count[task_id] = self._episode_count.get(task_id, 0) + 1
        
        result = EpisodeResult(
            episode_id=self._episode_count[task_id],
            total_reward=total_reward,
            steps=steps,
            duration_ms=duration_ms,
            success=success,
        )
        
        if task_id not in self._episode_history:
            self._episode_history[task_id] = []
        self._episode_history[task_id].append(result)
        
        # Decay epsilon
        if task_id in self._policies:
            self._policies[task_id].decay_epsilon()
        
        logger.info(f"Episode {result.episode_id} complete: "
                   f"reward={total_reward:.2f}, steps={steps}, success={success}")
        
        _bitacora(f"RL training: {task_id} episodes={result.episode_id}")
        
        return result
    
    def save_policy(self, task_id: str) -> bool:
        """Guarda politica a disco."""
        if task_id not in self._policies:
            return False
        
        if not self.storage_path:
            return False
        
        self.storage_path.mkdir(parents=True, exist_ok=True)
        filepath = self.storage_path / f"policy_{task_id}.json"
        
        try:
            self._policies[task_id].save(str(filepath))
            logger.info(f"Saved policy for task {task_id}")
            return True
        except Exception as e:
            logger.error(f"Error saving policy: {e}")
            return False
    
    def load_policy(self, task_id: str) -> bool:
        """Carga politica desde disco."""
        if not self.storage_path:
            return False
        
        filepath = self.storage_path / f"policy_{task_id}.json"
        if not filepath.exists():
            return False
        
        try:
            policy = Policy.load(str(filepath))
            self._policies[task_id] = policy
            self._buffers[task_id] = ReplayBuffer(self.buffer_capacity)
            logger.info(f"Loaded policy for task {task_id}")
            return True
        except Exception as e:
            logger.error(f"Error loading policy: {e}")
            return False
    
    def get_episode_stats(self, task_id: str, 
                         last_n: int = 100) -> Dict[str, Any]:
        """Obtiene estadisticas de episodios."""
        history = self._episode_history.get(task_id, [])
        recent = history[-last_n:] if history else []
        
        if not recent:
            return {"episodes": 0}
        
        return {
            "episodes": len(history),
            "recent_episodes": len(recent),
            "avg_reward": sum(e.total_reward for e in recent) / len(recent),
            "avg_steps": sum(e.steps for e in recent) / len(recent),
            "success_rate": sum(1 for e in recent if e.success) / len(recent),
            "best_reward": max(e.total_reward for e in recent),
        }
    
    def list_policies(self) -> List[str]:
        """Lista tareas con politicas."""
        return list(self._policies.keys())
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas globales."""
        return {
            "policies_count": len(self._policies),
            "total_episodes": sum(self._episode_count.values()),
            "buffer_sizes": {
                task_id: len(buf) 
                for task_id, buf in self._buffers.items()
            },
        }
